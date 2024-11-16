#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

constexpr int CLAMP = 'A';
constexpr int EXTEND = 'B';
constexpr int OPTICAL_PORT = 4;

// motor groups
pros::MotorGroup leftMotors({-20, 16, -15},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-17, -18, 19}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::MotorGroup intake({10}, pros::MotorGearset::blue);
pros::adi::DigitalOut clamp(CLAMP);
pros::adi::DigitalOut extend(EXTEND);
pros::MotorGroup arm({11}, pros::MotorGearset::green);
pros::Optical optical_sensor(OPTICAL_PORT);

// Inertial Sensor on port 10
pros::Imu imu(9);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
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
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
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

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
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

// Constants for motor speeds and timing
const int DRIVE_SPEED = 600;  // Speed at which the robot drives
const int LIFT_SPEED = 80;    // Speed at which the lift moves 


void autonomous() {
    leftMotors.move_velocity(DRIVE_SPEED);
    rightMotors.move_velocity(DRIVE_SPEED);
    pros::delay(20);  // Wait for 2000 milliseconds (2 seconds)

    // Stop driving
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);

    // Lift/claw picks up an object (raise lift for 1 second)
    arm.move_velocity(LIFT_SPEED);
    pros::delay(1000);  // Wait for 1 second
    arm.move_velocity(0);  // Stop the lift

    // Turn the robot right (by stopping the left motor and running the right motor)
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(DRIVE_SPEED);
    pros::delay(900);  // Wait for 900 milliseconds to complete a turn

    // Stop turning
    rightMotors.move_velocity(0);
    
    // Add more actions here if needed, such as additional movements or manipulations

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
      if (optical_sensor.get_hue() == 210)
      ;
        pros::delay(250);
        intake.move_velocity(0);
    }
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
    if (controller.get_digital(DIGITAL_R1)) {
      intake.move_velocity(600); // This is 600 because it's a 600rpm motor
    }
    else if (controller.get_digital(DIGITAL_R2)) {
      intake.move_velocity(-600);
    }
    else {
      intake.move_velocity(0);
    }

    if (controller.get_digital(DIGITAL_L1)) {
      arm.move_velocity(200); // This is 600 because it's a 600rpm motor
    }
    else if (controller.get_digital(DIGITAL_L2)) {
      arm.move_velocity(-200);
    }
    else {
      arm.move_velocity(0);
    }

    if (controller.get_digital(DIGITAL_X)) {clamp.set_value(true);}
    if (controller.get_digital(DIGITAL_B)) {clamp.set_value(false);}

    if (controller.get_digital(DIGITAL_Y)) {extend.set_value(true);}
    if (controller.get_digital(DIGITAL_A)) {extend.set_value(false);}
        pros::delay(10);
    }
}