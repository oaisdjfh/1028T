#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
//#include "pros/optical.hpp"
//#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>
#include <iostream>
pros::Controller controller(pros::E_CONTROLLER_MASTER);
	//pros::MotorGroup leftMotors({9,7,8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	//pros::MotorGroup rightMotors({-5,-3,-4});
    pros::MotorGroup rightMotors({9,7,8});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup leftMotors({-5,-3,-4});
	lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12, lemlib::Omniwheel::NEW_325, 450, 2); 
pros::Motor Intake(2);
pros::Motor Top(10);
pros::adi::DigitalOut middle ('d');
pros::adi::DigitalOut little_will ('a');
pros::adi::DigitalOut mid_descor('b');
pros::Rotation odomVert(6);
//pros::Rotation odomHorz(12);
pros::IMU imu(1);
pros::Distance front_distance(12);
pros::Distance side_distance(13);
bool will_val = false;
bool wing_val = false;
bool mid_descor_val = false;
pros::adi::DigitalOut wing('h');
lemlib::TrackingWheel vert_TrackingWheel(&odomVert, lemlib::Omniwheel::NEW_275, .45); //(&encoder name, wheeltype, offset)
//lemlib::TrackingWheel horz_TrackingWheel(&odomHorz, lemlib::Omniwheel::NEW_275, -2); //(&encoder name, wheeltype, offset)

//lemlib::OdomSensors sensors(&vert_TrackingWheel, nullptr,  &horz_TrackingWheel, nullptr, &imu);
lemlib::OdomSensors sensors(&vert_TrackingWheel, nullptr,  nullptr, nullptr, &imu);

int before_distance_x = 0;
int after_distance_x = 0;
int before_distance_y = 0;
int after_distance_y = 0;
int before_angle = 0;
int after_angle = 0;

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
lemlib::ControllerSettings angular_controller(2.6, // proportional gain (kP)
                                              0.2, // integral gain (kI)
                                              18, // derivative gain (kD)
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
    //odomHorz.reset_position();
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
            std::cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " Theta: " << chassis.getPose().theta << "         Intake velocity: " << Intake.get_actual_velocity() << "Front Distance: " << front_distance.get() << std::endl;
            // delay to save resources
            pros::delay(500);

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
 
 void rollers(double intake, double top){
	Intake.move(127*intake);
	Top.move(127*top);
}

void anti_jam(){
    for(int i = 0; i < 30; i++){
            if (Intake.get_actual_velocity() < 30 && Intake.get_actual_velocity() > -30){
                rollers(-1,0);
            }
            else{
                rollers(1,1);
            }
            pros::delay(100);
        }
    
}
//9,109,64
//68, 91, 117
void autonomous(){
    bool sawp = false;
    bool right = true;
    bool left = false;
    bool skills_old = false;
    bool skills_new = false;
    bool test = false;
    if (test){
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(90,1000,{.maxSpeed = 80, .minSpeed=40});
        }
    if(sawp){
        //SAWP
        
        chassis.setPose(0, 0, 0);
        rollers(1,0);
        chassis.moveToPoint(0,31, 1600,{.maxSpeed = 60, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.maxSpeed = 80, .minSpeed=40});
        little_will.set_value(1);
        chassis.moveToPose(20,30, 90, 1000, {.lead = .3, .minSpeed = 50});
        chassis.moveToPoint(-24, 31, 1500,{.forwards = false, .maxSpeed = 70});
        rollers(0,0);
        pros::delay(700);//time to start extake
        rollers(1,1);
        pros::delay(1100);//extake into right goal
        rollers(0,0);
        little_will.set_value(0);

        //start picking up 
        
        //chassis.moveToPose(-13, 24, 180, 1500, {.minSpeed = 50});
        chassis.moveToPose(-8, 27.5, 180, 1500, {.minSpeed = 50});
        pros::delay(300);
        rollers(1,1);
        pros::delay(500);
        rollers(1,0);
        //chassis.moveToPose(-32, 4, 230, 1500, {.minSpeed = 50});
        //chassis.turnToHeading(180,1000, {.maxSpeed = 80, .minSpeed = 50});
        //chassis.moveToPoint(-27.8, -36.8, 3000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.moveToPose(-26, 8, 230, 1500, {.minSpeed = 50});
        pros::delay(950);
        little_will.set_value(1);
        chassis.turnToHeading(180,1000, {.maxSpeed = 80, .minSpeed = 50});
        chassis.moveToPoint(-24, -40, 3000, {.maxSpeed = 80, .minSpeed = 40});
        little_will.set_value(0);



        //grab left corner balls
        pros::delay(800);
        little_will.set_value(1);
        chassis.turnToHeading(135, 1000, {.minSpeed = 50});
        //chassis.moveToPoint(-41, -24.5, 1500, {.forwards = false, .maxSpeed = 70,.earlyExitRange=3});
        chassis.moveToPoint(-41, -25, 1500, {.forwards = false, .maxSpeed = 70,.earlyExitRange=3});
        pros::delay(600);
        //score into middle goal
        middle.set_value(1);
        rollers(1,-1);
        pros::delay(1200);
        rollers(0,0);
        middle.set_value(0);

        //move to left side matchload
        rollers(1,0);
        /*
        chassis.moveToPoint(-6,-65,3000,{.maxSpeed = 75, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.minSpeed=50});
        chassis.moveToPoint(14,-65,1250);
        chassis.moveToPoint(-27, -65, 2000, {.forwards = false});
        */
        chassis.moveToPoint(-4,-66,3000,{.maxSpeed = 75, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.minSpeed=50});
        chassis.moveToPoint(20,-66,1250, {.maxSpeed = 60});
        chassis.moveToPoint(-23, -63, 2000, {.forwards = false, .maxSpeed = 70});
        pros::delay(750);
        rollers(1,1);
        while (true){
            if (Intake.get_actual_velocity() < 30){
                rollers(-1,0);
            }
            else{
                rollers(1,1);
            }
        }
    }
    

    if (right){
        chassis.setPose(0,0,0);
        rollers(1,0);
        wing.set_value(1);
        chassis.moveToPose(8,30,29,2000,{.minSpeed = 50});
        pros::delay(750);
        little_will.set_value(1);
        chassis.turnToHeading(137, 1000, {.minSpeed = 40});
        chassis.moveToPoint(29, 7, 2000, {.minSpeed = 40});
        little_will.set_value(0);
        chassis.turnToHeading(180, 1000, {.minSpeed = 40});
        little_will.set_value(1);
        chassis.moveToPoint(33, -15, 1400, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(33, 27, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        pros::delay(1000);
        little_will.set_value(0);
        rollers(1,1);
        pros::delay(2000);
        rollers(0,0);
        chassis.moveToPoint(33, 6, 1000, {.minSpeed = 40});
        chassis.turnToHeading(130, 1000, {.minSpeed = 40});
        chassis.moveToPoint(22, 25, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(180, 1000, {.minSpeed = 40});
        chassis.moveToPoint(22, 38, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        wing.set_value(0);
        /*
        chassis.setPose(0, 0, 0);
        rollers(1,0);
        chassis.moveToPose(-32, 4, 270, 1500, {.minSpeed = 50});
        chassis.moveToPose(0,31, 0, 1600,{.maxSpeed = 60, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.maxSpeed = 80, .minSpeed=40});
        little_will.set_value(1);
        chassis.moveToPoint(10,32, 1250, {.minSpeed = 50});
        chassis.moveToPoint(-26, 32, 1500,{.forwards = false});
        rollers(0,0);
        pros::delay(700);//time to start extake
        rollers(1,1);
        pros::delay(900);//extake into right goal
        rollers(0,0);
        little_will.set_value(0);


        chassis.moveToPose(-13, 24, 180, 1500, {.minSpeed = 50});
        pros::delay(300);
        rollers(1,1);
        pros::delay(500);
        rollers(1,0);
*/
    }



    if(left){
        //Left side
        //-1.77, -32.24, 180 facing to left matchload
        //-1.8, -32, -90 facing to middle goal
        chassis.setPose(0,0,0);
        rollers(1,0);
        wing.set_value(1);
        chassis.moveToPose(-7,25,-28, 1500,{.minSpeed = 50});
        pros::delay(750);
        little_will.set_value(1);
        chassis.turnToHeading(-135, 1000, {.minSpeed = 40});
        chassis.moveToPose(5, 37, -135, 1000,{.forwards = false, .lead = .3, .minSpeed = 40});
        little_will.set_value(0);
        chassis.waitUntilDone();
        middle.set_value(1);
        rollers(1,-1);
        pros::delay(2000);
        rollers(0,0);
        middle.set_value(0);
        //grab left matchload
        chassis.moveToPose(-32, -1, -135, 2000, {.minSpeed = 40});
        chassis.turnToHeading(180, 1000, {.minSpeed = 40});
        little_will.set_value(1);
        rollers(1,0);
        chassis.moveToPoint(-33, -10, 1000, {.minSpeed = 40});
        chassis.moveToPoint(-33, 20, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        pros::delay(750);
        rollers(1,1);
        pros::delay(2000);
        chassis.moveToPoint(-34, 8, 1000, {.minSpeed = 40});
        chassis.turnToHeading(-215, 1000, {.minSpeed = 40});
        chassis.moveToPoint(-42,20,1000,{.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(180, 1000, {.minSpeed = 40});
        chassis.moveToPoint(-42, 37, 3000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        wing.set_value(0);

    }


    if(skills_new){
        chassis.setPose(0,0,0);
        wing.set_value(1);
        rollers(1,0);
        chassis.moveToPoint(-4, 16.8, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.waitUntilDone();
        pros::delay(100);
        rollers(0,0);
        chassis.moveToPoint(-11.4, 25, 1000,{.minSpeed = 40});
        chassis.turnToHeading(-135, 1000, {.minSpeed = 40});
        chassis.moveToPose(3, 42, 135, 1000,{.forwards = false, .lead = .3, .minSpeed = 40});
        chassis.waitUntilDone();
        pros::delay(300);
        middle.set_value(1);
        rollers(1,-1);
        pros::delay(2000);
        middle.set_value(0);


        //grab left matchload
        chassis.moveToPoint(-32, 5, 2000, {.maxSpeed = 60});
        chassis.waitUntilDone();
        chassis.turnToHeading(180, 1000);
        little_will.set_value(1);
        rollers(1,0);
        chassis.moveToPose(-32, -20, 180, 2500, {.minSpeed = 70});
        chassis.waitUntilDone();
        chassis.moveToPoint(-32, -3, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPose(-45.3, 19.7, 180, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});


        //go to other side of field
        chassis.moveToPoint(-42, 89, 4000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40});
        little_will.set_value(0);
        rollers(0,0);
        chassis.turnToHeading(270,2000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-33, 91, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(345, 2000, { .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-33, 75,1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        pros::delay(750);
        rollers(1,1);
        pros::delay(2500);
        rollers(1,0);
        little_will.set_value(1);
        chassis.moveToPoint(-30, 110,3000, {.maxSpeed = 70, .minSpeed = 50});
        chassis.moveToPoint(-32, 75, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        pros::delay(1000);
        /*
        rollers(1,1);
        pros::delay(2500);
        rollers(1,0);
        */
        anti_jam();
        rollers(1,0);
        little_will.set_value(0);
        chassis.moveToPose(-16, 102, 45, 2000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPose(1, 112, 90, 2500, {.maxSpeed = 60, .minSpeed = 40});
        
        chassis.waitUntilDone();
        chassis.turnToHeading(90,1000);
        before_distance_x = front_distance.get();
        before_distance_y = side_distance.get();
        little_will.set_value(1);
        chassis.moveToPoint(45, 122, 3000, {.minSpeed = 70});
        
        chassis.waitUntilDone();
        pros::delay(200);
        chassis.turnToHeading(90,1000);
        after_distance_x = front_distance.get();
        after_distance_y = side_distance.get();

        int distance_traveled_inch_x = ((before_distance_x - after_distance_x))/25.4;
        int distance_traveled_inch_y = ((before_distance_y - after_distance_y))/25.4;
        //
        //
        chassis.setPose(1+distance_traveled_inch_x, 112+distance_traveled_inch_y, chassis.getPose().theta);
    }




    if(skills_old){

        chassis.setPose(0, 0, 0);
        rollers(1,0);
        chassis.moveToPoint(0,31, 1600,{.maxSpeed = 80, .minSpeed=40, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.maxSpeed = 80, .minSpeed=40});
        little_will.set_value(1);
        chassis.moveToPoint(10,32, 2250, {.maxSpeed = 70, .minSpeed = 50});
        chassis.moveToPoint(-26, 32, 1500,{.forwards = false});
        rollers(0,0);
        pros::delay(700);//time to start extake
        rollers(1,1);
        pros::delay(4000);//extake into right goal
        rollers(0,0);
        little_will.set_value(0);

        //start picking up 
        
        chassis.moveToPose(-13, 24, 180, 1500, {.minSpeed = 50});
        pros::delay(300);
        rollers(1,1);
        pros::delay(500);
        rollers(1,0);
        chassis.moveToPose(-32, 4, 230, 1500, {.minSpeed = 50});
        pros::delay(500);
        little_will.set_value(1);
        chassis.turnToHeading(180,1000, {.maxSpeed = 80, .minSpeed = 50});
        //chassis.moveToPoint(-30, -37, 3000, {.maxSpeed = 80, .minSpeed = 40});
        little_will.set_value(0);
        chassis.moveToPoint(-27, -36.8, 3000, {.maxSpeed = 80, .minSpeed = 40});

        //grab left corner balls
        pros::delay(800);
        little_will.set_value(1);
        chassis.turnToHeading(135, 1000);
        //chassis.moveToPoint(-38.5, -25.8, 1500, {.forwards = false, .maxSpeed = 70,.earlyExitRange=3});
        chassis.moveToPoint(-41, -24.5, 1500, {.forwards = false, .maxSpeed = 70,.earlyExitRange=3});
        pros::delay(600);
        //score into middle goal
        middle.set_value(1);
        rollers(.5,-.5);
        pros::delay(2650);
        rollers(0,0);
        middle.set_value(0);
        

        //move to left side matchload
        rollers(1,0);
        chassis.moveToPoint(-6,-65,3000,{.maxSpeed = 75, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.minSpeed=50});
        chassis.moveToPoint(14,-65,3250);
        chassis.moveToPoint(0, -65, 1000, {.minSpeed = 40});
        chassis.turnToHeading(-135, 1000, {.minSpeed = 40});
        little_will.set_value(0);
        chassis.moveToPose(-23.5, -77.4, 270, 1000, {.minSpeed = 40});
        chassis.moveToPose(-82, -77.4, 270, 3000, {.minSpeed = 40});



        chassis.moveToPose(-102, -60.5, -90, 1000, {.minSpeed = 40});
        chassis.moveToPose(-83, -61, -90, 1000, {.forwards = false, .minSpeed = 40});
        pros::delay(500);
        rollers(1,1);
        pros::delay(3000);
        rollers(0,0);
        chassis.moveToPoint(-115, -60.4, 3000,{.minSpeed = 40});
        chassis.moveToPose(-83, -61, -90, 1000, {.forwards = false, .minSpeed = 40});
        pros::delay(500);
        rollers(1,1);
        pros::delay(3000);
        rollers(0,0);
        chassis.moveToPoint(-102, -60.5, 1000, {.minSpeed = 40});
        




        /*
        chassis.moveToPose(-100, -65, -90, 1000, {.minSpeed = 40});
        chassis.moveToPose(-81, -65, -90, 1000, {.forwards = false, .minSpeed = 40});
        pros::delay(500);
        rollers(1,1);
        pros::delay(3000);
        rollers(0,0);
        chassis.moveToPoint(-115, -65, 3000,{.minSpeed = 40});
        chassis.moveToPose(-81, -65, -90, 1000, {.forwards = false, .minSpeed = 40});
        pros::delay(500);
        rollers(1,1);
        pros::delay(3000);
        rollers(0,0);
        chassis.moveToPoint(-100, -65, 1000, {.minSpeed = 40});
        */



        /*
        //Skills
        chassis.setPose(0, 0, 0);
        rollers(1,0);
        chassis.moveToPoint(0,32, 1600,{.maxSpeed = 60, .minSpeed=40, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.maxSpeed = 60, .minSpeed=40});
        little_will.set_value(1);
        chassis.moveToPoint(10,32, 3000);
        chassis.moveToPoint(-24, 32, 1500,{.forwards = false});
        rollers(0,0);
        pros::delay(700);//time to start extake
        rollers(1,1);
        pros::delay(2500);//extake into right goal
        rollers(0,0);
        little_will.set_value(0);

        //start picking up 
        
        chassis.moveToPose(-13, 24, 180, 1500, {.minSpeed = 50});
        pros::delay(300);
        rollers(1,1);
        pros::delay(500);
        rollers(1,0);
        chassis.moveToPose(-32, 4, 230, 1500, {.minSpeed = 50});
        chassis.turnToHeading(180,1000, {.maxSpeed = 60, .minSpeed = 50});
        chassis.moveToPoint(-30, -37, 3000, {.maxSpeed = 60, .minSpeed = 40});

        //grab left corner balls
        pros::delay(700);
        little_will.set_value(1);
        chassis.turnToHeading(135, 1000, {.minSpeed = 50});
        //chassis.moveToPoint(-45, -27, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange=3});
        chassis.moveToPoint(-42, -28.4, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 30, .earlyExitRange=3});
        pros::delay(600);
        //score into middle goal
        middle.set_value(1);
        rollers(1,1);
        pros::delay(2000);
        rollers(0,0);
        middle.set_value(0);

        //move to left side matchload
        rollers(1,0);
        chassis.moveToPoint(-6,-67.5,3000,{.maxSpeed = 75, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.minSpeed=50});
        chassis.moveToPoint(8,-67.5,3000);

        //move other side
        chassis.moveToPoint(-3, -67, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(-300, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-17,-80,1000,{.forwards = false, .minSpeed = 40});
        chassis.turnToHeading(90,1000,{.maxSpeed = 60, .minSpeed = 40});
        wing.set_value(1);
        chassis.moveToPose(-31, -83, 90, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-98, -84, 3000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPose(-107, -75.6, 180, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPose(-101, -73, 270, 1000, {.forwards = false,.maxSpeed = 60,  .minSpeed = 40});
        chassis.moveToPoint(-85, -67, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        //extake into left goal
        pros::delay(700);
        rollers(1,1);
        pros::delay(2000);
        rollers(1,0);
        //intake form left back matchload
        chassis.moveToPoint(-120, -67, 2000);
        chassis.moveToPoint(-85, -67, 2000, {.forwards = false});
        pros::delay(700);
        rollers(1,1);
        pros::delay(2000);
        rollers(1,0);
        little_will.set_value(0);
        //move to right side
        chassis.moveToPose(-103, -48, 0, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-103, 30, 3000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(270, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-120, 30, 2000);
        chassis.moveToPoint(-86, 31, 2000, {.forwards = false});
        pros::delay(700);
        rollers(1,1);
        pros::delay(2000);
        rollers(0,0);
        chassis.moveToPoint(-103, 32, 2000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(315, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(-86, 15, 1000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPose(13, 14, 270, 3000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.turnToHeading(180, 1000, {.maxSpeed = 60, .minSpeed = 40});
        chassis.moveToPoint(12, -12, 2000);
    }
    */



        /*
        chassis.setPose(0, 0, 0);
        rollers(1,0);
        chassis.moveToPoint(0,32, 1600,{.maxSpeed = 60, .minSpeed=40, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.maxSpeed = 60, .minSpeed=40});
        little_will.set_value(1);
        chassis.moveToPoint(14,32, 5500);
        chassis.moveToPoint(-26, 32, 1500,{.forwards = false});
        rollers(0,0);
        pros::delay(700);//time to start extake
        rollers(1,1);
        pros::delay(3000);//extake into right goal
        rollers(0,0);
        little_will.set_value(0);

        //start picking up 
        
        chassis.moveToPose(-13, 24, 180, 1500, {.minSpeed = 50});
        pros::delay(300);
        rollers(1,1);
        pros::delay(500);
        rollers(1,0);
        chassis.moveToPose(-32, 4, 230, 1500, {.minSpeed = 50});
        chassis.turnToHeading(180,1000, {.maxSpeed = 60, .minSpeed = 50});
        chassis.moveToPoint(-30, -37, 3000, {.maxSpeed = 60, .minSpeed = 40});

        //grab left corner balls
        pros::delay(700);
        little_will.set_value(1);
        chassis.turnToHeading(135, 1000, {.minSpeed = 50});
        //chassis.moveToPoint(-45, -27, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange=3});
        chassis.moveToPoint(-42, -28.4, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 30, .earlyExitRange=3});
        pros::delay(600);
        //score into middle goal
        middle.set_value(1);
        rollers(1,1);
        pros::delay(3500);
        rollers(0,0);
        middle.set_value(0);

        //move to left side matchload
        rollers(1,0);
        chassis.moveToPoint(-6,-75,3000,{.maxSpeed = 75, .minSpeed=30, .earlyExitRange=10});
        chassis.turnToHeading(90,1000,{.minSpeed=50});
        chassis.moveToPoint(10,-75,5250);
        chassis.moveToPoint(-27, -75, 2000, {.forwards = false});
        pros::delay(500);
        rollers(1,1);
        pros::delay(3000);
        little_will.set_value(0);
        chassis.moveToPose(1.3, -61.5, -305, 2000);
        chassis.moveToPose(15, -36.6, 0, 2000);
        chassis.moveToPoint(15, -15, 2000, {.minSpeed = 60});
        little_will.set_value(1);

        */
        /*
        chassis.setPose(0,0,0);
        rollers(-1,0);
        chassis.moveToPoint(0, -10, 1000, {.forwards = false});
        chassis.moveToPoint(0,10,3000, {.minSpeed = 80});
        */
    } 

}
pros::Task little_task_thing([]() {
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
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
pros::Task mid_descor_task_thing([]() {
    while (true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            mid_descor_val = !mid_descor_val;
            mid_descor.set_value(mid_descor_val);
            pros::delay(500);
        }
    }
    pros::delay(20);
});
void opcontrol() {
    //wing.set_value(1);
	while (true) {
        //pros::Task little_task_handle(little_task);
        //pros::Task wing_task_handle(wing_task);
        
        int rightX = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(rightX, leftY);

        pros::delay(20);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			rollers(1,0);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            rollers(1,1);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            middle.set_value(1);
            rollers(.8,-.6);
            //rollers(1,-1);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            rollers(-1,-1);
        }else {
			rollers(0, 0);
			middle.set_value(0);
        }
    
    }
}