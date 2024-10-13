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
pros::MotorGroup left_motors({-18, -17, -16}, pros::MotorGearset::green); // left motors on ports 11, 12, 13; all reverse
pros::MotorGroup right_motors({13, 14, 15}, pros::MotorGearset::green); // right motors on ports 18, 19, 20; all forwards

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

//Distance Sensor
pros::Distance right_distance_sensor(8); //Right
pros::Distance back_distance_sensor(7); //Back

//v5 Rotation Sensor
//pros::Rotation horizontal_encoder(14);

//Horizontal Tracking Wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::OLD_275, 0);


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
    6, // proportional gain (kP)
    0, // integral gain (kI)
    14, // derivative gain (kD)
    0, // anti windup
    0.5, // small error range, in inches
    250, // small error range timeout, in milliseconds
    2, // large error range, in inches
    500, // large error range timeout, in milliseconds
    127 // maximum acceleration (slew)
);

// Angular PID Controller
lemlib::ControllerSettings angular_controller(
    2.4, // proportional gain (kP)
    0, // integral gain (kI)
    15, // derivative gain (kD)
    0, // anti windup
    0.5, // small error range, in degrees
    250, // small error range timeout, in milliseconds
    2, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    127 // maximum acceleration (slew)
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
pros::adi::DigitalOut tipMech(6);

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

void RedRightRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // move backwards to  mobile goal
    chassis.moveToPoint(0, -36, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-6, -48, 2000, {.forwards = false, .maxSpeed = 40}, false);
    // pick up mobile goal and run intake to score ring
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    chassis.turnToPoint(-12, -26, 2000, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(750);
        intake.brake();      
        }
    };
    chassis.moveToPoint(-12, -26, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-10, 0, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-10, -18, 2000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(250);
    backClaw.set_value(false);
    chassis.moveToPoint(-10, -32, 2000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.moveToPoint(10, -32, 2000, {.forwards = false, .maxSpeed = 90}, false);
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    pros::delay(1000);
    intake.brake();
    chassis.moveToPoint(28, -46, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void RedRightSigSoloWP() {
    chassis.setPose(0, 0, 0);
    //chassis.moveToPoint(0, -12, 2000, {.forwards = false, .maxSpeed = 127}, false);
    //chassis.turnToPoint(8, -12, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(0, -10, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(8, -16, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(-8, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-22, -14, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task2{[=] {
        pros::delay(250);
        intake.brake();        
        }
    };
    chassis.moveToPoint(-34, 8, 1000, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(true);
    chassis.turnToPoint(-34, 38, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-34, 38, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(250);
    pros::Task task3{[=] {
        pros::delay(1000);
        intake.move_voltage(-12000);        
        }
    };
    chassis.turnToPoint(-30, 50, 1500, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(100);
    chassis.moveToPoint(-32, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-51, 30, 1500, {.forwards = false, .maxSpeed = 90}, false);
    backClaw.set_value(true);
    pros::delay(100);
    intake.brake();
    chassis.moveToPoint(-6, 42, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(6, 46, 750, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(750);
    hang.set_value(true);
    chassis.moveToPoint(-48, -2, 2000, {.forwards = false, .maxSpeed = 127}, false);
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
    chassis.moveToPoint(-6, -24, 2000, {.forwards = false, .maxSpeed = 90}, false);
    pros::delay(250);
    // run intake
    intake.move_voltage(12000);
    pros::delay(500);
    // move to and intake red rings close to middle line
    chassis.turnToPoint(8, -38, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(8, -38, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(26, -40, 2000, {.forwards = true, .maxSpeed = 50}, false);
    chassis.moveToPoint(4, -40, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(16, -24, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(-18, -34, 2000, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    chassis.moveToPoint(-18, -34, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void RedLeftSimpleSoloWP() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -8, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-8, -14, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(8, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(20, -14, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task2{[=] {
        pros::delay(250);
        intake.brake();        
        }
    };
    chassis.moveToPoint(38, 8, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(100);
    backClaw.set_value(true);
    chassis.turnToPoint(38, 34, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(38, 34, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(100);
    chassis.moveToPoint(38, 6, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(51, 20, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(44, 10, 1000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.moveToPoint(52, 24, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(46, 12, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(750);
    chassis.moveToPoint(50, -12, 2000, {.forwards = false, .maxSpeed = 127}, false);

}

void BlueLeftRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    backClaw.set_value(false);
    // move backwards to  mobile goal
    chassis.moveToPoint(0, -36, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(6, -48, 2000, {.forwards = false, .maxSpeed = 40}, false);
    // pick up mobile goal and run intake to score ring
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    chassis.turnToPoint(10, -24, 2000, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(750);
        intake.brake();      
        }
    };
    chassis.moveToPoint(10, -24, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(12, 0, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(12, -16, 2000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(250);
    backClaw.set_value(false);
    chassis.moveToPoint(12, -28, 2000, {.forwards= true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-10, -28, 2000, {.forwards = false, .maxSpeed = 90}, false);
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    pros::delay(1000);
    intake.brake();
    chassis.moveToPoint(-24, -38, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void BlueRight() {
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
    chassis.moveToPoint(6, -24, 2000, {.forwards = false, .maxSpeed = 90}, false);
    pros::delay(250);
    // run intake
    intake.move_voltage(12000);
    pros::delay(500);
    // move to and intake red rings close to middle line
    chassis.turnToPoint(-4, -42, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-4, -42, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-26, -46, 2000, {.forwards = true, .maxSpeed = 50}, false);
    chassis.moveToPoint(0, -46, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-16, -30, 2000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    intake.brake();
    chassis.turnToPoint(20, -44, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(20, -44, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void BlueRightElim() {
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
    chassis.moveToPoint(6, -24, 2000, {.forwards = false, .maxSpeed = 90}, false);
    pros::delay(250);
    // run intake
    intake.move_voltage(12000);
    pros::delay(500);
    // move to and intake red rings close to middle line
    chassis.turnToPoint(-4, -42, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-4, -42, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-26, -48, 2000, {.forwards = true, .maxSpeed = 50}, false);
    chassis.moveToPoint(0, -48, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-18, -16, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(20, -44, 2000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(250);
    chassis.moveToPoint(80, 0, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void BlueRightSimpleWP() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -8, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(8, -15, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(-8, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, -14, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task2{[=] {
        pros::delay(250);
        intake.brake();        
        }
    };
    chassis.moveToPoint(-34, 6, 1000, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(true);
    pros::delay(100);
    chassis.turnToPoint(-34, 32, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-34, 32, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(100);
    chassis.moveToPoint(-34, 10, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-46, 24, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(-40, 16, 1000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.moveToPoint(-48, 30, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    chassis.moveToPoint(-46, 10, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::delay(750);
    hang.set_value(true);
    chassis.moveToPoint(-50, -8, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void Skills() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, -4, 0);
    // move back to alliance stake
    //chassis.moveToPoint(0, -4, 2000, {.forwards = false, .maxSpeed = 30}, false);
    // run intake to score preload on alliance stake
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(250);
    intake.brake();
    // move to mobile goal on left
    chassis.moveToPoint(0, 14, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-22, 14, 1500, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(750);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-22, 14, 1500, {.forwards = false, .maxSpeed = 60}, false);
    // move to intake red ring
    chassis.turnToPoint(-24, 36, 1500, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-24, 36, 1500, {.forwards = true, .maxSpeed = 60}, false);
    // move to intake red ring near wall stake
    chassis.turnToPoint(-56, 58, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-56, 58, 1500, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.turnToPoint(-44, 60, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-44, 60, 1500, {.forwards = false, .maxSpeed = 60}, false);
    // move to intake three red rings in line
    /*while (back_distance_sensor.get() < 1600 && back_distance_sensor.get() > 575) { 
        left_motors.move_voltage((-(1600-back_distance_sensor.get()))*15);
        right_motors.move_voltage((-(1600-back_distance_sensor.get()))*15);
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();*/
    chassis.turnToPoint(-46, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-46, 0, 2000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(1000);
    chassis.turnToPoint(-58, 14,1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-58, 14, 1500, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(500);
    // drop mobile goal in corner
    chassis.turnToPoint(-64, 0, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-64, 0, 1500, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    backClaw.set_value(false);
    pros::delay(250);
    // move to mobile goal on other side
    chassis.moveToPoint(-50, 12, 1500, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.turnToPoint(18, 10, 1500, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(1350);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(18, 10, 1500, {.forwards = false, .maxSpeed = 90}, false);
    // move to intake red ring
    chassis.turnToPoint(16, 28, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(16, 28, 1500, {.forwards = true, .maxSpeed = 60}, false);
    // move to intake red ring near wall stake
    chassis.turnToPoint(46, 56, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(46, 56, 1500, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.turnToPoint(38, 58, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(38, 58, 1500, {.forwards = false, .maxSpeed = 60}, false);
    // move to intake three red rings in line
    chassis.turnToPoint(40, 0, 2000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(40, 0, 2000, {.forwards = true, .maxSpeed = 45}, false);
    /*while (back_distance_sensor.get() < 1600 && back_distance_sensor.get() > 575) { 
        left_motors.move_voltage((-(1600-back_distance_sensor.get()))*15);
        right_motors.move_voltage((-(1600-back_distance_sensor.get()))*15);
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();*/
    pros::delay(1000);
    chassis.turnToPoint(52, 14, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(52, 14, 1500, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(500);
    // drop mobile goal in corner
    chassis.turnToPoint(56, -4, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(56, -4, 1500, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    backClaw.set_value(false);
    pros::delay(250);
    // move to blue alliance side
    chassis.moveToPoint(40, 66, 1500, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.turnToPoint(16, 84, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(16, 84, 1500, {.forwards = true, .maxSpeed = 60}, false);
    intake.brake();
    chassis.turnToPoint(-2, 98, 1500, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task3{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(750);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-2, 98, 1500, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.turnToPoint(54, 98, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(54, 98, 1500, {.forwards = true, .maxSpeed = 60}, false);
    //chassis.turnToPoint(30, 98, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(30, 98, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(42, 112, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(42, 112, 1500, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(0, 52, 1500, {.forwards = true, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    intake.brake();
    chassis.moveToPoint(60, 120, 1500, {.forwards = false, .maxSpeed = 127}, false);
    /*left_motors.move_voltage(-4500);
    right_motors.move_voltage(-4500);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();*/
    hang.set_value(true);
    chassis.moveToPoint(0, 52, 1500, {.forwards = true, .maxSpeed = 90}, false);
    //chassis.moveToPoint(-60, 110, 2000, {.forwards = true, .maxSpeed = 127}, false);
}

void Test() {
    while (back_distance_sensor.get() < 1200){ //Go to matchload bar code
        left_motors.move_voltage((-(1200-back_distance_sensor.get()))*20);
        right_motors.move_voltage((-(1200-back_distance_sensor.get()))*20);
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();
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
            // print measurements from the rotation sensor
            //pros::lcd::print(3, "Rotation Sensor: %i", horizontal_encoder.get_position());
            // print measurements from the distance sensors
            pros::lcd::print(3, "rightDistanceSensor: %f", fmt::to_string((right_distance_sensor.get())));
            pros::lcd::print(4, "backDistanceSensor: %f", fmt::to_string((back_distance_sensor.get())));
            // delay to save resources 
            pros::delay(50);
        }
    });
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

/*
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
    RedRightSigSoloWP();
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

    hang.set_value(false);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool backClawOpened = false;
    bool backClawClosed = true;
    bool hangUp = false;
    bool hangDown = true;
    bool tipMechOpened = false;
    bool tipMechClosed = true;

    // loop forever

    while (true) {
        //Tank drive
        // get left y and right y positions
        /*int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);*/

        //Arcade drive
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
    

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(-12000);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(12000);
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

        /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) and tipMechOpened) {
            if (tipMechClosed) {
                tipMech.set_value(false);
                tipMechClosed = false;
            }
            else if (not tipMechClosed and tipMechOpened){
                tipMech.set_value(true);
                tipMechClosed = true;
            }
            tipMechOpened = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            tipMechOpened = true;
        }*/

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) and hangUp) {
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

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
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