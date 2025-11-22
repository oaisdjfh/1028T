#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
//#include "pros/optical.hpp"
//#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <iostream>
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftFront(1); // Left front motor
pros::Motor leftMid(3); // Left front motor
pros::Motor leftBack(2); // Left front motor
pros::Motor rightFront(12);  // Left back motor
pros::Motor rightMid(11); // Right front motor
pros::Motor rightBack(13); // Right back motor
	pros::MotorGroup leftMotors({-11,-12,-13});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup rightMotors({3,2,1});
	lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_325, 450, 2); 
pros::Motor Intake(14);
pros::Motor Top(10);
pros::adi::DigitalOut middle ('d');
pros::adi::DigitalOut little_will ('a');
pros::Rotation odomVert(-16);
pros::Rotation odomHorz(-9);
pros::IMU imu(19);
bool skills = false;
bool left = false;
bool will_val = false;
bool wing_val = false;
pros::adi::DigitalOut wing('h');
lemlib::TrackingWheel vert_TrackingWheel(&odomVert, lemlib::Omniwheel::NEW_2, -.25); //(&encoder name, wheeltype, offset)
lemlib::TrackingWheel horz_TrackingWheel(&odomHorz, lemlib::Omniwheel::NEW_2, -1.75); //(&encoder name, wheeltype, offset)

lemlib::OdomSensors sensors(&vert_TrackingWheel, nullptr,  &horz_TrackingWheel, nullptr, &imu);


// lateral PID controllerf

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.7, // proportional gain (kP)
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
    chassis.setPose(0, 0, 0);
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screaen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            std::cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " Theta: " << chassis.getPose().theta << std::endl;
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
            pros::delay(1000);

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
 
 void rollers(int intake, double top){
	Intake.move(127*intake);
	Top.move(127*top);
}

void autonomous(){
    /*
    chassis.setPose(0, 0, 0);
    rollers(-1,0);

    //chassis.moveToPose(-6.69,30.54,-21.60,3000,{.maxSpeed=70, .minSpeed=50});
    chassis.moveToPose(-9.6,30.2,-28.6,3000,{.maxSpeed=70, .minSpeed=50});
    chassis.turnToHeading(-131.64, 1000);
    chassis.moveToPose(3.5,38,-131.64,1500,{.forwards = false, .maxSpeed=80, .earlyExitRange=5});
    

    //-3.1,29,-82
    chassis.waitUntilDone();
    middle.set_value(1);
    rollers(-1,-1);
    pros::delay(2500);
    rollers(0,0);
    middle.set_value(0);

    chassis.moveToPose(-33,-.86,-132.9,4000,{.minSpeed=60});
    little_will.set_value(1);
    rollers(-1,0);
    chassis.turnToHeading(-180,1000);
    chassis.moveToPoint(-31,-18,2000,{.maxSpeed=60, .minSpeed=50});
    pros::delay(1200);
    rollers(0,0);

    chassis.moveToPoint(-34, 20, 1300,{.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    rollers(-1,1);
    pros::delay(200);
    if (Intake.get_actual_velocity()<50){
        rollers(1,-1);
        pros::delay(300);
    }
    rollers(-1,1);
    */
    
    chassis.setPose(0,0,0);
    rollers(-1,0);
    chassis.moveToPose(12.2,30.4,42.8,2000,{.maxSpeed=70, .minSpeed=50});
    chassis.turnToHeading(159.7,1000);
    chassis.moveToPoint(34,-3.2, 3000);
    chassis.turnToHeading(190,1000);
    chassis.moveToPoint(34, 18, 1000, {.forwards=false});
    chassis.waitUntilDone();
    rollers(-1,1);
    pros::delay(3000);
    chassis.moveToPoint(34, 10, 2000);
    /*
    little_will.set_value(1);
    chassis.moveToPose(27.4, .9, 173.2, 3000,{.minSpeed=60});
    chassis.moveToPose(28.3, -30, 180, 2000,{.minSpeed=65});    
    pros::delay(2000);
    rollers(0,0);
    chassis.moveToPose(31,21,180,1500,{.forwards = false});
    chassis.waitUntilDone();
    rollers(-1,1);
    pros::delay(200);
    if (Intake.get_actual_velocity()<10){
        rollers(1,-1);
        pros::delay(300);
    }
    rollers(-1,1);
    */
}

void little_task(){
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            will_val = !will_val;
            little_will.set_value(will_val);
            pros::delay(500);
        }
    }
    pros::delay(20);
}
void wing_task(){
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            wing_val = !wing_val;
            wing.set_value(wing_val);
            pros::delay(500);
        }
    }
    pros::delay(20);
}
pros::Task little_task_thing([]() {
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            will_val = !will_val;
            little_will.set_value(will_val);
            pros::delay(500);
        }
    }
    pros::delay(20);
});
pros::Task wing_task_thing([]() {
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            wing_val = !wing_val;
            wing.set_value(wing_val);
            pros::delay(500);
        }
    }
    pros::delay(20);
});
        
void opcontrol() {

	while (true) {
        //pros::Task little_task_handle(little_task);
        //pros::Task wing_task_handle(wing_task);
        
        int rightX = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(rightX, leftY);

        pros::delay(20);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			rollers(-1,-.2);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            rollers(-1,1);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            middle.set_value(1);
            rollers(-1,1);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            rollers(1,-1);
        }else {	
			rollers(0, 0);
			middle.set_value(0);
        }
	   }
}