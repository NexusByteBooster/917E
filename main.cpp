#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <future>

// controller introduction
pros::Controller controller(pros::E_CONTROLLER_MASTER);

constexpr int CLAMP = 'A'; // constant for our mogo mech port
constexpr int OPTICAL_PORT = 4; // constant for our optical sensor for color sorting99
// motor groups
pros::MotorGroup leftMotors({11, -12, -14}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-15, 16, 18 }, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor intake(19, pros::MotorGearset::blue);
pros::adi::DigitalOut clamp(CLAMP);
pros::Motor arm(11);
pros::Rotation rotation_sensor(8);

pros::Optical optical_sensor(OPTICAL_PORT);

// Inertial Sensor on port 12

pros::Imu imu(13);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // chassis rpm is 360
                              6 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where chassis will move out of 127
                                     1.53 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where chassis will move out of 127
                                  1.5 // expo curve gain
);

// chassis settings
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);



// create the chassis

/** 
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    chassis.calibrate();
    chassis.setPose(0,0,0);

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("chassis pose: {}", chassis.getPose());
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


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

// Constants for motor speeds and timing
const int DRIVE_SPEED = 600;  // Speed at which the robot drives
const int LIFT_SPEED = 100;    // Speed at which the lift moves 

void autonomous() {
    // Stop driving
    chassis.setPose(0, -4, 0);
    intake.move_velocity(600);
    pros::delay(1100);
    chassis.moveToPoint(0, 7, 1100);
    clamp.set_value(false);
    chassis.turnToHeading(270, 1100, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(21, 7, 2000, {.forwards = false, .maxSpeed = 90}, true);
    chassis.waitUntil(21);
    clamp.set_value(true);
    pros::delay(2000);
    chassis.moveToPoint(20, 7, 1000);                                                                                                             
    chassis.turnToHeading(0, 1100);
    chassis.moveToPoint(20, 30, 1100);
    chassis.turnToHeading(60, 1100);
    chassis.moveToPoint(48, 48, 1500);
    chassis.moveToPoint(39, 45, 1100, {.forwards = false});
    chassis.turnToHeading(180, 1500);
    chassis.moveToPoint(39, 18, 2000);
    chassis.moveToPoint(39, 4, 2500);
    chassis.moveToPoint(39, -1, 2500);
    chassis.moveToPoint(39, 10, 3500, {.forwards = false});
    chassis.turnToHeading(90, 1100, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(42, 10, 2000);
    chassis.moveToPoint(39, 10, 2000, {.forwards = false});
    chassis.turnToHeading(350, 1100);
    chassis.moveToPoint(51, -8, 1100, {.forwards = false});
    pros::delay(1000);
    clamp.set_value(false);
    pros::delay(1000);
    chassis.moveToPoint(36, 16, 1500);
    chassis.turnToHeading(358, 1000);
    clamp.set_value(false);
    chassis.moveToPoint(36, 3, 1000, {.forwards = false});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-12, 3, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.moveToPoint(-31, 3, 2000, {.forwards = false, .maxSpeed = 90}, true);
    chassis.waitUntil(19);
    clamp.set_value(true);
    pros::delay(3000);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(-42, 3, 1500);
    chassis.moveToPoint(-48, 3, 1500);
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(-48, 44, 2000);
    chassis.moveToPoint(-54, -6, 2000, {.forwards = false});
    pros::delay(1200);
    clamp.set_value(false);
    pros::delay(1720);
    chassis.moveToPoint(-48, 30, 1500);
    chassis.turnToHeading(50, 1500);
    clamp.set_value(false);
    chassis.moveToPoint(0, 120, 2000);
    chassis.turnToHeading(90, 1500);
    chassis.moveToPoint(-48, 120, 4500);
    chassis.moveToPoint(-54, 120, 5000);

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X );
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
      arm.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
      if (controller.get_digital(DIGITAL_R1)){
        intake.move_velocity(590); // This is 600 because it's a 600rpm motor
      }
      else if (controller.get_digital(DIGITAL_R2)) {
        intake.move_velocity(-590);
      }
      else {
        intake.move_velocity(0);
      }

      if (controller.get_digital(DIGITAL_L1)) {
        arm.move_velocity(300); // This is 600 because it's a 600rpm motor
        intake.move_velocity(-590);
        pros::delay(100);
        intake.move_velocity(0);
      }
      else if (controller.get_digital(DIGITAL_L2)) {
        arm.move_velocity(-300);
      }
      else {
        arm.move_velocity(0);
      }
      if (controller.get_digital(DIGITAL_Y)) {clamp.set_value(true);}
      if (controller.get_digital(DIGITAL_B)) {clamp.set_value(false);}
      intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
      int hue = optical_sensor.get_hue();  // Get hue value from the optical sensor 
        if (hue > 200 && hue < 260) {  // Blue object detected
          pros::delay(20);
          intake.move_velocity(0);
        }

        // Delay to prevent excessive polling of the optical sensor
        pros::delay(50);
    }
}