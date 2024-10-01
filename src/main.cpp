#include "main.h"
    #include "fmt/format.h"
#include "lemlib/pose.hpp"
    #include "main.h"
    #include "pros/adi.hpp"
    #include "pros/colors.h"
    #include "pros/misc.h"
    #include "pros/motors.h"
    #include "pros/rtos.hpp"
    #include "pros/screen.hpp"
    #include "lemlib/api.hpp"
#include <algorithm>
    #include <chrono>
    #include <string>
    #include <iostream>
    #include <type_traits>
	#include <cmath> 

// Drivetrain Motors
pros::MotorGroup left_motors({-18, -19, -20}, pros::MotorGearset::green); // left motors on ports 1, 2, 3; all forwards
pros::MotorGroup right_motors({11, 12, 13}, pros::MotorGearset::green); // right motors on ports 4, 5, 6; all forwards

// Drivetrain Settings
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
    10.5, // 11 inch track width
    lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);

//Inertial Sensor
pros::Imu inertial_sensor(1);

//Final Configuration of sensors
lemlib::OdomSensors sensors(
	nullptr,
    nullptr, 
    nullptr, // horizontal tracking wheel 1
	nullptr,
    &inertial_sensor // inertial sensor
);

// PID Constants
// Lateral PID Controller
lemlib::ControllerSettings lateral_controller(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    50, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    250, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// Angular PID Controller
lemlib::ControllerSettings angular_controller(
    2.5, // proportional gain (kP)
    0.15, // integral gain (kI)
    18, // derivative gain (kD)
    10, // anti windup
    0.5, // small error range, in degrees
    200, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    600, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// Create the Chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
	sensors // odometry sensors
);

pros::Motor intake(12);
pros::adi::DigitalOut backClaw(8);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void RedSoloWPNonRush() {
  // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(-10, -2, 2500, {.forwards = false}, false);
    chassis.moveToPoint(5, 10, 2500, {.forwards = true}, false);
    chassis.moveToPoint(5, 40, 2500, {.forwards = false}, false);
    chassis.moveToPoint(-6, 20, 2500, {.forwards = true}, false);
    left_motors.move_voltage(3000);
    right_motors.move_voltage(3000);
    pros::delay(750);
    left_motors.brake();
    right_motors.brake();
    chassis.moveToPoint(18, 50, 2000, {.forwards = false}, false);
    chassis.moveToPoint(0, 42, 2000, {.forwards = true}, false);
}

void RedRightRush() {
    chassis.setPose(0, 0, 0);
    backClaw.set_value(true);
    chassis.moveToPoint(-4, -30, 2500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-11, -48, 2500, {.forwards = false, .maxSpeed = 40}, false);
    backClaw.set_value(false);
    intake.move_voltage(12000);
    pros::delay(250);
    backClaw.set_value(true);
    intake.brake();
    chassis.moveToPoint(-4, -32, 2500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(10, -32, 2500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(10, -32, 2500, {.forwards = false, .maxSpeed = 60}, false);
    backClaw.set_value(false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-28, -32, 2500, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(12, -14, 2500, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    backClaw.set_value(true);
    chassis.moveToPoint(12, -14, 2500, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(28, -6, 2500, {.forwards = true, .maxSpeed = 30}, false);
    chassis.turnToPoint(26, 8, 2500, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    chassis.moveToPoint(26, 8, 2500, {.forwards = false, .maxSpeed = 60}, false);
    intake.move_voltage(12000);
    pros::delay(250);
    left_motors.move_voltage(12000);
    right_motors.move_voltage(12000);

    /*left_motors.move_voltage(-4000);
    right_motors.move_voltage(-3000);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();
    chassis.moveToPoint(-10, -34, 2000, {.forwards = true}, false);
    left_motors.move_voltage(3000);
    right_motors.move_voltage(3000);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();
    chassis.moveToPoint(16, -14, 2000, {.forwards = true}, false);
    left_motors.move_voltage(3000);
    right_motors.move_voltage(3000);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();
    chassis.moveToPoint(20, 8, 2000, {.forwards = false}, false);
    left_motors.move_voltage(12000);
    right_motors.move_voltage(12000);
    pros::delay(750);
    left_motors.brake();
    right_motors.brake();*/


}

void RedLeft() {
    chassis.setPose(0, 0, 0);
    chassis.turnToPoint(0, -12, 2000, {.forwards = false}, false);
    chassis.moveToPoint(0, -12, 2000, {.forwards = false}, false);
    left_motors.move_voltage(-3000);
    right_motors.move_voltage(-3000);
    pros::delay(1000);
    left_motors.brake();
    right_motors.brake();
    chassis.turnToPoint(16, -22, 2000, {.forwards = true}, false);
    chassis.moveToPoint(16, -22, 2000, {.forwards = true}, false);
    left_motors.move_voltage(3000);
    right_motors.move_voltage(3000);
    pros::delay(750);
    left_motors.brake();
    right_motors.brake();
    left_motors.move_voltage(-3000);
    chassis.turnToPoint(4, -22, 2000, {.forwards = false}, false);
    chassis.moveToPoint(4, -22, 2000, {.forwards = false}, false);
    chassis.turnToHeading(135, 2000);

}

void initialize() {
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
void autonomous() {
    // Auton Function
    RedLeft();
}

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
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);
    }
}