#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftFront(8); // Left front motor
pros::Motor leftMid(9); // Left front motor
pros::Motor leftBack(10); // Left front motor
pros::Motor rightFront(6);  // Left back motor
pros::Motor rightMid(7); // Right front motor
pros::Motor rightBack(5); // Right back motor
	pros::MotorGroup leftMotors({-15,-16,-17});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup rightMotors({5,7,6});
	lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_325, 450, 2); 
pros::Motor Intake(18);
pros::Motor back(19);
pros::Motor Snail(20);
pros::Motor Top(11);
pros::adi::DigitalOut seesaw ('a');
pros::adi::DigitalOut exitTop ('b');
pros::adi::DigitalOut tube ('c');
pros::adi::DigitalOut descore ('d');
pros::Rotation odomVert(-1);
pros::Rotation odomHorz(14);
pros::IMU imu(12);
pros::Optical color_sensor(2);

//1 is team red, 0 is team blue
int team = 1;
bool skills = false;
bool tube_val = false;
bool descore_val = false;
bool left = false;
bool color_sort_on = false;

lemlib::TrackingWheel vert_TrackingWheel(&odomVert, lemlib::Omniwheel::NEW_2, -1); //(&encoder name, wheeltype, offset)
lemlib::TrackingWheel horz_TrackingWheel(&odomHorz, lemlib::Omniwheel::NEW_2, 2); //(&encoder name, wheeltype, offset)

lemlib::OdomSensors sensors(&vert_TrackingWheel, nullptr,  &horz_TrackingWheel, nullptr, &imu);


// lateral PID controllerf

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              100, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/*
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    left = !left;
    show_left = abs(show_left-1);
  }
}
*/
void initialize() {
    odomHorz.reset_position();
	odomVert.reset_position();
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    chassis.setPose(0, 0, 0);
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, left ? "Left Side" : "Right Side");
            if (pros::lcd::read_buttons() == 28) {
                pros::lcd::print(3, "Confirm?");
                pros::delay(500);
                while (true){
                    if (pros::lcd::read_buttons() == 28) {
                        left = !left;
                        pros::delay(300);
                        break;
                    }
                    else if (pros::lcd::read_buttons() == 26){
                        break;
                    }
                    pros::delay(20);
                }
            }
            printf("Buttons Bitmap: %d\n", pros::lcd::read_buttons());
            // delay to save resources
            pros::delay(20);

        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


/*
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 
 void rollers(int intake, int Back, int top, int snail){
	Intake.move(127*intake);
	back.move(127*Back);
	Top.move(127*top);
	Snail.move(127*snail);
}

void quarter_auton(int x_inverse, int y_inverse, int shift_angle){
    exitTop.set_value(1);
    rollers(1,-1,-1,0);
    chassis.setPose(0, 0, 0);
    seesaw.set_value(1);


    //get balls
    chassis.moveToPose(26,22.6,-45+shift_angle,2000);

    //put balls in middle goal
    chassis.moveToPose(-1,51,-45+shift_angle,2000);//move to lower middle goal
    chassis.waitUntilDone();
    rollers(-1, -1, -1, -1);//put ball in goal
    pros::delay(1500);
    rollers(0,0,0,0);

    
    //move back
    leftMotors.move(-127);
    rightMotors.move(-127);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    
    
    //go grab balls in matchload
    chassis.moveToPose(32,16,175,1500, {.lead = .4});
    chassis.waitUntilDone();
    tube.set_value(1);
    pros::delay(200);

    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);

    //chassis.moveToPose(33,5,180,1800);//go to tube
    chassis.waitUntilDone();

    //chassis.waitUntil(2000);
    rollers(1,-1,-1,0);
    pros::delay(1000);
    rollers(0,0,0,0);

    //move back
    leftMotors.move(-127);
    rightMotors.move(-127);
    pros::delay(200);
    leftMotors.move(0);
    rightMotors.move(0);

    //put balls in side goal
    tube.set_value(0);
    //chassis.moveToPose(-25-shift,-49,90,2000);//go to big goal
    chassis.moveToPose(31,32,0,2000);//go to big goal
    chassis.waitUntilDone();
    seesaw.set_value(0);
    rollers(1,-1,-1,-1);
    pros::delay(6000);
    rollers(0,0,0,0);
}
void autonomous(){
    if (!skills){
        int flip = 1;
        int shift_angle = 0;
        if (left){
            flip = -1;
            shift_angle = 90;
        }
        quarter_auton(1,flip,shift_angle);
    } else{
        quarter_auton(1, 1, 0);
        chassis.moveToPose(-5, 10, 270, 3000);
        quarter_auton(1,-1,90);
    }
}
void opcontrol() {
	while (true) {
        exitTop.set_value(1);
        color_sensor.set_led_pwm(50);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // // move the chassis with curvature drive
        // chassis.arcade(leftY, rightX);
        // // delay to save resources
        // pros::delay(10);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        chassis.arcade(rightX, leftY);

        pros::delay(20);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			rollers(1, -1, -1, 0);
            if (color_sort_on){
                if (team){
                    if (color_sensor.get_hue()<10){seesaw.set_value(1); pros::delay(200);}
                    else{seesaw.set_value(0);}
                }
                else{
                    if (color_sensor.get_hue()>150){seesaw.set_value(1); pros::delay(200);}
                    else{seesaw.set_value(0);}
                }
            } else{
                seesaw.set_value(1);
            }
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			rollers(1, -1, 1, -1);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            tube.set_value(!tube_val);
			tube_val = !tube_val;
			pros::delay(500);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			rollers(1, -1, 1, 0);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            //color_sort_on = !color_sort_on;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			rollers(1, -1, -1, -1);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			rollers(1, -1, -1, 1);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			rollers(-1, -1, -1, -1);
		}
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            rollers(-1, 1,1,-1);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            descore.set_value(!descore_val);
			descore_val = !descore_val;
			pros::delay(500);
        }
        else {	
			rollers(0, 0, 0, 0);
			seesaw.set_value(0);
        }
		pros::delay(10);
	   }
}