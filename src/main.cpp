#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
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
pros::Rotation odomVert(14);
pros::Rotation odomHorz(1);
pros::IMU imu(12);
bool tube_val = false;



lemlib::TrackingWheel vert_TrackingWheel(&odomVert, lemlib::Omniwheel::NEW_2, -1); //(&encoder name, wheeltype, offset)
lemlib::TrackingWheel horz_TrackingWheel(&odomHorz, lemlib::Omniwheel::NEW_2, -4); //(&encoder name, wheeltype, offset)

lemlib::OdomSensors sensors(&vert_TrackingWheel, nullptr,  &horz_TrackingWheel, nullptr, &imu);


// lateral PID controllerf

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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
void initialize() {
    odomHorz.reset_position();
	odomVert.reset_position();
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
            printf("X: %.2f, Y: %.2f, theta: %.2f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

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
/*
ASSET(example_txt);

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.follow(example_txt, 15, 2000);
}
*/
void autonomous(){
    chassis.setPose(0,0,0);
    
}

void opcontrol() {
	while (true) {
        exitTop.set_value(1);


        //         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // // move the chassis with curvature drive
        // chassis.arcade(leftY, rightX);
        // // delay to save resources
        // pros::delay(10);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        /*
        FILE* usd_file_write = fopen("inputs.txt", "w");
        fputs(power, usd_file_write);
        fputs(turn, usd_file_write);
        */
        pros::delay(20);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			rollers(1, -1, -1, 0);
			seesaw.set_value(1);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			rollers(1, -1, 1, -1);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            tube.set_value(!tube_val);
			tube_val = !tube_val;
			pros::delay(500);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			rollers(1, -1, 1, 0);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			rollers(1, -1, -1, 0);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			rollers(1, -1, -1, -1);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			rollers(1, -1, -1, 1);
		} else {	
			rollers(0, 0, 0, 0);
			seesaw.set_value(0);
        }

		pros::delay(10);
	   }
}