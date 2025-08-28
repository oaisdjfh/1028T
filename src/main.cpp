#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
// #include "lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftFront(8); // Left front motor
pros::Motor leftMid(9); // Left front motor
pros::Motor leftBack(10); // Left front motor
pros::Motor rightFront(6);  // Left back motor
pros::Motor rightMid(7); // Right front motor
pros::Motor rightBack(5); // Right back motor

	pros::MotorGroup leftMotors({15,16,17});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup rightMotors({5,7,6}); 
pros::Motor Intake(18);
pros::Motor back(20);
pros::Motor Snail(19);
pros::Motor Top(11);
pros::Imu imu(11);
// pros::Rotation horizontalEnc(17);
// pros::Rotation verticalEnc(18);
pros::adi::DigitalOut seesaw ('a');
pros::adi::DigitalOut exitTop ('b');
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -3.25);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, .5);

// // drivetrain settings
// lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
//                               &rightMotors, // right motor group
//                               14, // 10 inch track width
//                               lemlib::Omniwheel::OLD_325, // using new 4" omnis
//                               450, // drivetrain rpm is 360
//                               2 // horizontal drift is 2. If we had traction wheels, it would have been 8
// );

// // lateral motion controller
// lemlib::ControllerSettings linearController(7, // proportional gain (kP)
//                                             0, // integral gain (kI)
//                                             10, // derivative gain (kD)
//                                             0, // anti windup
//                                             1, // small error range, in inches
//                                             100, // small error range timeout, in milliseconds
//                                             3, // large error range, in inches
//                                             500, // large error range timeout, in milliseconds
//                                             20 // m // maximum acceleration (slew)
// );

// // angular motion controller
// lemlib::ControllerSettings angularController(2.7, // proportional gain (kP)
//                                              0, // integral gain (kI)
//                                              20, // derivative gain (kD)
//                                              0, // anti windup
//                                              0, // small error range, in degrees
//                                              0, // small error range timeout, in milliseconds
//                                              0, // large error range, in degrees
//                                              0, // large error range timeout, in milliseconds
//                                              0 // maximum acceleration (slew)
// );

// // sensors for odometry
// lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
//                             &horizontal, // horizontal tracking wheel
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

// // input curve for throttle input during driver control
// lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

// // create the chassis
// lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

 // Creates a motor group with forwards port 5 and reversed ports 4 & 6

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {


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
void autonomous() {}

/**
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
void opcontrol() {
	   while (true) {

exitTop.set_value(1);
        //         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // // move the chassis with curvature drive
        // chassis.arcade(leftY, rightX);
        // // delay to save resources
        // pros::delay(10);
        int power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        leftMotors.move(-power - turn);
        rightMotors.move(power - turn);

        pros::delay(20);


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
         				Intake.move(127);
			back.move(-127);
			Top.move(-127);
			seesaw.set_value(1);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            // Intake.move(-127);
			// back.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			Intake.move(127);
			back.move(-127);
			Top.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			Intake.move(127);
			back.move(-127);
			Top.move(-127);
        // } else if (intakeOverride == 1){
        //     Intake.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
						Intake.move(127);
			back.move(-127);
			Top.move(-127);
			Snail.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			Intake.move(127);
			back.move(-127);
			Top.move(-127);
			Snail.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			Intake.move(127);
			back.move(-127);
			Top.move(-127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
						Intake.move(127);
			back.move(-127);
			Top.move(-127);
			seesaw.set_value(1);
		} else {	
            Intake.move(0);
			back.move(0);
			Top.move(0);
			Snail.move(0);
			seesaw.set_value(0);
        }

		pros::delay(10);
	   }
}