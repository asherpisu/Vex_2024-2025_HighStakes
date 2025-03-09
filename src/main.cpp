#include "main.h"
#include <sys/types.h>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.h"

#define OPTICAL_PORT 3

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-20, -13, -19}, pros::MotorGearset::blue); //left group
pros::MotorGroup rightMotors({18, 14, 9}, pros::MotorGearset::blue); //right group
pros::Motor lb(-10); 
pros::Motor intake(-17);
//pneumatics
pros::adi::DigitalOut clamp('H', false);
pros::adi::DigitalOut doinker('A');
// Inertial Sensor on port 1
pros::Imu imu(4);
//Rotation Sensor for lb on port 16
pros::Rotation lbRotation(-1);
//Optical Sensor on port 3 (CHANGE AS NEEDED)
pros::Optical colorSensor(OPTICAL_PORT);
//lb code
bool isSpinning = false;
int target = 0;
int load = 19;
double kp = 3.75;

void scoreLb(){
    //kp = 5.0;
    intake.move_relative(-150, 100);      
    target = 145;
               
}

void liftLb(){
    intake.move_relative(-150, 100);  
    target = 45;     
}

void loadLb(){
    //kp = 3.8;
    if (target != load){
        target = load;
    }    
    else{
        target = 0;
    }
    //intake.move_velocity(600);   
}

void descore(){
    if (target == 150){
        target = load;
    }
    else{
        target = 150; 
    }
}

void flip(){
    if (target != 200){
        target = 200;
    }
    else{
        target = load;
    }
}


void liftControl(){                
    lb.move(kp*(target-(lbRotation.get_position()/100.0)));    
}

void backwardsIntake(){
    intake.move_relative(-150, 100);  
}

bool intake_spinning = false;
const double INTAKE_MIN_VELOCITY = 50.0;
const double CURRENT_THRESHOLD = 1.5;
/*
void antiJam() {
    // Only check for jam if intake was previously spinning above minimum velocity
    if (!intake_spinning) return;

    // Get motor state
    double current = intake.get_current_draw();
    double velocity = std::abs(intake.get_actual_velocity());
    double target_velocity = std::abs(intake.get_target_velocity());
    
    // Detect if motor is completely stopped while trying to move
    if (target_velocity > 0 && 
        velocity <=10 && 
        current > CURRENT_THRESHOLD &&
        target == load) {
        // Stop the motor to prevent damage
        pros::delay(10);       
        intake.move_velocity(0);
        intake_spinning = false;  // Indicate intake is no longer spinning                                        
    }
}
*/

void colorSort(bool againstRed, bool toggleOn) {
    double color = colorSensor.get_hue();
    const int delayTime = 20;
    const int stopTime = 1;
    if (toggleOn) {
        if (againstRed) {
            if (color > 350 || (color > 10 & color < 60)) {
                pros::delay(delayTime);
                intake.move_velocity(0);
                pros::delay(stopTime);
                intake.move_velocity(600);
            }
        }
        else {
            if (color > 190 & color < 290) {
                pros::delay(delayTime);
                intake.move_velocity(0);
                pros::delay(stopTime);
                intake.move_velocity(600);
            }
        }
    }
}

void closeClamp(){
    clamp.set_value(false);
}

void openClamp(){
    clamp.set_value(true);
    intake.move_relative(-150, 100);  
}

void doinkerDown(){
    doinker.set_value(true);
}

void doinkerUp(){
    doinker.set_value(false);
}


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-6);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(5);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4.58);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -1);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.625, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(3.3, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.4 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.4 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lbRotation.set_position(0);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);    
    pros::Task backGroundTasks([]{
        while (true){
            liftControl();
            //antiJam();
            //if (std::abs(intake.get_actual_velocity()) > INTAKE_MIN_VELOCITY) {
                intake_spinning = true;
            //}     
            colorSort(true, true);                  
            pros::delay(10);                    
        }
    });    
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
*/
void autonomous() {
      // set position to x:0, y:0, heading:0
      chassis.setPose(0, 0, 0);
      // move 48" forwards
      chassis.moveToPoint(0, 24, 10000);
}

/**
 * Runs in driver control
 */


void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the robot
        chassis.tank(leftY, rightY, true);        
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            closeClamp();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            openClamp();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            doinkerDown();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            doinkerUp();
        }             
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            backwardsIntake();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            liftLb();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            flip();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            loadLb();            
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            scoreLb();            
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            intake.move_velocity(600);
            //pros::delay(1000);
            // Check if motor reaches minimum velocity to start tracking
                                             
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            intake.move_velocity(0);   
            intake_spinning = false;
        }
        // delay to save resources
        pros::delay(10);
    }
}