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
pros::MotorGroup left_motors({-18, -17, -16}, pros::MotorGearset::green); // left motors on ports 1, 2, 3; all forwards
pros::MotorGroup right_motors({13, 14, 15}, pros::MotorGearset::green); // right motors on ports 4, 5, 6; all forwards

// Drivetrain Settings
lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
    11.5, // 11 inch track width
    lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);

//Inertial Sensor
pros::Imu inertial_sensor(19);

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
    9, // proportional gain (kP)
    0, // integral gain (kI)
    70, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in inches
    250, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// Angular PID Controller
lemlib::ControllerSettings angular_controller(
    1.75, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in degrees
    250, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
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
pros::adi::DigitalOut hang(7);

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

void RedRightRushNoWP() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(true);
    chassis.moveToPoint(-3, -30, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-11, -48, 2000, {.forwards = false, .maxSpeed = 40}, false);
    backClaw.set_value(false);
    pros::delay(250);
    intake.move_voltage(12000);
    pros::delay(250);
    backClaw.set_value(true);
    intake.brake();
    chassis.moveToPoint(-4, -24, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    chassis.turnToPoint(9, -24, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(9, -24, 2000, {.forwards = false, .maxSpeed = 60}, false);
    backClaw.set_value(false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-22, -30, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(12, -14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    backClaw.set_value(true);
    chassis.moveToPoint(12, -14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(30, 0, 2000, {.forwards = true, .maxSpeed = 30}, false);
    intake.brake();
    chassis.moveToPoint(22, 3, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.moveToPoint(30, 6, 2000, {.forwards = false, .maxSpeed = 60}, false);
    intake.move_voltage(12000);
}

void RedRightRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(false);
    // move backwards to  mobile goal
    chassis.moveToPoint(-5, -40, 2500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-10, -46, 2500, {.forwards = false, .maxSpeed = 40}, false);
    // pick up mobile goal and run intake to score ring
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    pros::delay(750);
    // drop mobile goal
    backClaw.set_value(false);
    intake.brake();
    // move to next mobile goal
    chassis.moveToPoint(0, -42, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(18, -32, 2000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(1000);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(18, -32, 2000, {.forwards = false, .maxSpeed = 50}, false);
    // run intake
    intake.move_voltage(12000);
    // move to intake ring
    chassis.moveToPoint(-22, -32, 2000, {.forwards = true, .maxSpeed = 60}, false);
    // move to touch ladder
    chassis.moveToPoint(18, -42, 2000, {.forwards = false, .maxSpeed = 60}, false);
}

void RedLeft() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(false);
    // move  backwards to mobile goal
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(1000);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-6, -27, 2000, {.forwards = false, .maxSpeed = 90}, false);
    pros::delay(250);
    // run intake
    intake.move_voltage(12000);
    // move to and intake red rings close to middle line
    chassis.turnToPoint(8, -36, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(8, -36, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(20, -36, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-2, -40, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(12, -22, 2000, {.forwards = true, .maxSpeed = 60}, false);
    // move to stack of rings near alliance stake
    chassis.turnToPoint(-26, -8, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-26, -8, 2000, {.forwards = true, .maxSpeed = 90}, false); 
    // intake blue ring   
    pros::delay(250);
    intake.brake();
    // outtake blue ring in different direction
    chassis.moveToPoint(-26, 4, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    pros::delay(250);
    chassis.moveToPoint(-26, -10, 2000, {.forwards = false, .maxSpeed = 127}, false);
    // move back and intake red ring
    intake.move_voltage(12000);
    chassis.moveToPoint(-48, 6, 2000, {.forwards = true, .maxSpeed = 90}, false);
}

void BlueLeftRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(false);
    // move backwards to  mobile goal
    chassis.moveToPoint(4, -40, 2500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(13, -45, 2500, {.forwards = false, .maxSpeed = 40}, false);
    // pick up mobile goal and run intake to score ring
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    pros::delay(750);
    // drop mobile goal
    backClaw.set_value(false);
    intake.brake();
    // move to next mobile goal
    chassis.moveToPoint(2, -42, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, -28, 2000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(1000);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-20, -28, 2000, {.forwards = false, .maxSpeed = 50}, false);
    // run intake
    intake.move_voltage(12000);
    // move to intake ring
    chassis.moveToPoint(18, -30, 2000, {.forwards = true, .maxSpeed = 60}, false);
    // move to touch ladder
    chassis.moveToPoint(-20, -42, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::Task task2{[=] {
        // stop intake
        pros::delay(1500);
        intake.brake();
        }
    };

}

void BlueRight() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(false);
    // move backwards to mobile goal
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(1000);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(6, -27, 2000, {.forwards = false, .maxSpeed = 90}, false);
    pros::delay(250);
    // run intake
    intake.move_voltage(12000);
    // move to and intake blue rings close to middle line
    chassis.turnToPoint(-8, -42, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-8, -42, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-24, -42, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(2, -42, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-12, -22, 2000, {.forwards = true, .maxSpeed = 60}, false);
    // move to ladder
    chassis.turnToPoint(24, -38, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(24, -38, 2000, {.forwards = false, .maxSpeed = 127}, false);
    
    /*// move to stack of rings near alliance stake
    chassis.turnToPoint(30, -4, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(30, -4, 2000, {.forwards = true, .maxSpeed = 90}, false); 
    // intake red ring   
    pros::delay(250);
    intake.brake();
    // outtake red ring in different direction
    chassis.moveToPoint(28, -2, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    pros::delay(500);
    chassis.moveToPoint(24, -16, 2000, {.forwards = false, .maxSpeed = 127}, false);
    // move back and intake red ring
    intake.move_voltage(12000);
    chassis.turnToPoint(44, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(44, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);*/
}

void Skills() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // move back to alliance stake
    chassis.moveToPoint(0, -4, 2000, {.forwards = false, .maxSpeed = 30}, false);
    // run intake to score preload on alliance stake
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(250);
    intake.brake();
    // move to mobile goal on left
    chassis.moveToPoint(0, 14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-22, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-22, 14, 2000, {.forwards = false, .maxSpeed = 60}, false);
    backClaw.set_value(true);
    // move to intake red rings
    chassis.turnToPoint(-36, 4, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-36, 4, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-20, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-52, 14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-52, 14, 2000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(-18, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, 32, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, 32, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(-52, 32, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-52, 32, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(-56, 56, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-56, 56, 2000, {.forwards = true, .maxSpeed = 60}, false);
    pros::Task task1{[=] {
        // outtake while robot is still moving
        pros::delay(1500);
        intake.move_voltage(-12000);
        }
    };
    chassis.moveToPoint(-58, 0, 2000, {.forwards = false, .maxSpeed = 127}, false);
    // drop mobile goal in corner
    backClaw.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(-50, 14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(20, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(20, 14, 2000, {.forwards = false, .maxSpeed = 90}, false);
    backClaw.set_value(true);
    intake.brake();
    chassis.turnToPoint(42, 4, 2000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(42, 4, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(18, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(54, 14, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(54, 14, 2000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(18, 14, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(20, 32, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(20, 32, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(52, 32, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(52, 32, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(56, 56, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(56, 56, 2000, {.forwards = true, .maxSpeed = 60}, false);
    

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
    //Set Motors to Brake
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(50);
    // Auton Function
    Skills();
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

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool backClawOpened = false;
    bool backClawClosed = true;
    bool hangUp = false;
    bool hangDown = true;
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(-12000);
        }
        else {
                intake.move_voltage(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and backClawOpened) {
            if (backClawClosed) {
                backClaw.set_value(false);
                backClawClosed = false;
            }
            else if (not backClawClosed and backClawOpened){
                backClaw.set_value(true);
                backClawClosed = true;
            }
            backClawOpened = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            backClawOpened = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) and hangUp) {
            if (hangDown) {
                hang.set_value(false);
                hangDown = false;
            }
            else if (not hangDown and hangUp){
                hang.set_value(true);
                hangDown = true;
            }
            hangUp = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            hangUp = true;
        }

        /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            backClaw.set_value(true);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            backClaw.set_value(false);
        }*/

         // delay to save resources
        pros::delay(25);

    }
}