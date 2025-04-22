
#include "main.h"
    #include "fmt/format.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
    #include "main.h"
    #include "pros/adi.hpp"
    #include "pros/colors.h"
    #include "pros/misc.h"
    #include "pros/motors.h"
    #include "pros/rtos.hpp"
    #include "pros/imu.hpp"
    #include "pros/screen.hpp"
    #include "lemlib/api.hpp"
#include <algorithm>
    #include <chrono>
    #include <string>
    #include <iostream>
    #include <type_traits>
	#include <cmath> 

// Drivetrain Motors
pros::MotorGroup left_motors({-19, -20, -9}, pros::MotorGearset::green); // left motors on ports 11, 12, 13; all reverse
pros::MotorGroup right_motors({13, 12, 2}, pros::MotorGearset::green); // right motors on ports 18, 19, 20; all forwards

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
pros::Imu inertial_sensor(18);

//Distance Sensor
pros::Distance distance_sensor(6); //Right
//pros::Distance back_distance_sensor(7); //Back

//Optical Sensor
pros::Optical optical_sensor(10);

// Rotation Sensor
pros::Rotation rotation_sensor(7);
pros::Rotation tracking_pod(5);

// Line Tracker
pros::adi::AnalogIn line_sensor('A');

//Horizontal Tracking Wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::OLD_275, 0);

//Vertical Tracking Wheel
lemlib::TrackingWheel vertical_tracking_wheel(&tracking_pod, lemlib::Omniwheel::NEW_2, 0);

//Final Configuration of sensors
lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1
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
    63, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in inches
    250, // small error range timeout, in milliseconds
    2, // large error range, in inches
    500, // large error range timeout, in milliseconds
    127 // maximum acceleration (slew)
);

/*lemlib::ControllerSettings lateral_controller(
    0, // proportional gain (kP)
    0, // integral gain (kI)
    0, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);*/

// Angular PID Controller
lemlib::ControllerSettings angular_controller(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    12, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in degrees
    250, // small error range timeout, in milliseconds
    2, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    127 // maximum acceleration (slew)
);

/*lemlib::ControllerSettings angular_controller(
    0, // proportional gain (kP)
    0, // integral gain (kI)
    0, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in degrees
    0, // small error range timeout, in milliseconds
    0, // large error range, in degrees
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);*/

// Create the Chassis
lemlib::Chassis chassis(
	drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
	sensors // odometry sensors
);

//pros::Motor ladybrown_1(-1, pros::MotorGearset::green);
//pros::Motor ladybrown_2(8, pros::MotorGearset::green);   

pros::MotorGroup ladybrown({1, -8}, pros::MotorGearset::green);
pros::Motor intake(11);
pros::adi::DigitalOut backClaw(7);
pros::adi::DigitalOut stick(6);
pros::adi::DigitalOut tip(2);
pros::adi::DigitalOut stickClaw(8);

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

const int numStates = 3;
int states[numStates] = {5, 3000,14200};
int currentState = 0;
int target = 5;
int error;
void nextState() {
    currentState++;
    if (currentState == 3) {
        currentState = 0;
    }
    target = states[currentState];  
}

const int numofIntakeStates = 2;
int intakeStates[numofIntakeStates] = {5, 7000};
int currentIntakeState = 0;
void intakeState() {
    currentIntakeState++;
    if (currentIntakeState == 2) {
        currentIntakeState = 0;
    }
    target = intakeStates[currentIntakeState];
}

const int numofDescoreStates = 2;
int descoreStates[numofDescoreStates] = {5, 15200};
int currentDescoreState = 0;
void descoreState() {
    currentDescoreState++;
    if (currentDescoreState == 2) {
        currentDescoreState = 0;
    }
    target = descoreStates[currentDescoreState];
}

void liftControl() {
    double kp = 0.008;
    double error = target - rotation_sensor.get_position();
    double velocity = kp * error;
    ladybrown.move(velocity);
    /*while (rotation_sensor.get_position() != states[target]){ //Go to matchload bar code
        ladybrown.move_voltage((states[target] - rotation_sensor.get_position())*10);
        pros::delay(20);
    }*/
}

bool intakeStop = false;
void redColorSort() {
        int ring_hue = optical_sensor.get_hue();
        if (ring_hue < 240 && ring_hue > 210) {
            //intakeStop = true;
            pros::delay(100);
            intake.move_voltage(-12000);
            pros::delay(50);
            intake.move_voltage(12000);
        }
        //intakeStop = false;
}

void blueColorSort() {
    int ring_hue = optical_sensor.get_hue();
        if (ring_hue < 20 && ring_hue > 0) {
            //intakeStop = true;
            pros::delay(100);
            intake.move_voltage(-12000);
            pros::delay(50);
            intake.move_voltage(12000);
        }
        //intakeStop = false;
}

int jamTime = 0;
void unjam() {
    int jamCheck = intake.get_actual_velocity();
    int voltage = intake.get_voltage();
    if (jamCheck < 1000 && voltage > 8000) {
        jamTime += 20;
        if (jamTime > 100) {
            intake.move_voltage(-12000);
            pros::delay(1000);
            intake.move_voltage(12000);
            jamTime = 0;
        }
        else {
            jamTime = 0;
        }
    }
}

void lineTrack() {
    while (true) {
        int sensorValue = line_sensor.get_value();

    }
}

void RedRightRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    intake.move_voltage(12000);
    pros::Task task1{[=] {
        stick.set_value(true);
        pros::delay(800);
        stickClaw.set_value(true);
    }
    };
    chassis.moveToPoint(0, 40, 900, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(550);
        intake.brake();
    }
    };
    chassis.moveToPoint(0, 24, 1000, {.forwards = false, .maxSpeed = 127}, false);
    stickClaw.set_value(false);
    pros::delay(250);
    stick.set_value(false);
    pros::delay(250);
    chassis.turnToPoint(-18, 20, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task3{[=] {
        pros::delay(800);
        backClaw.set_value(true);
        pros::delay(250);
        intake.move_voltage(12000);
    }
    };
    chassis.moveToPoint(-18, 20, 2000, {.forwards = false, .maxSpeed = 45}, false);
    pros::delay(250);
    intake.brake();
    backClaw.set_value(false);
    chassis.moveToPoint(-2, 14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-8, 34, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task4{[=] {
        pros::delay(750);
        backClaw.set_value(true);
    }
    };
    chassis.moveToPoint(-8, 34, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task5{[=] {
        pros::delay(1000);
        intake.move_voltage(12000);
    }
    };
    chassis.moveToPoint(46, -20, 1750, {.forwards = true, .maxSpeed = 60} ,false);
    chassis.turnToPoint(0, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(0, 0, 1500, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);
    backClaw.set_value(false);
    chassis.turnToPoint(-20, 8, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task6{[=] {
        pros::delay(800);
        backClaw.set_value(true);
        }
    };
    chassis.moveToPoint(-20, 8, 1000, {.forwards = false, .maxSpeed = 127}, false);
}

void RedLeftSimpleWP() {
    chassis.setPose(0, -8, 0);
    //chassis.moveToPoint(0, -9, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-10, -14, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(8, -14, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(900);
        intake.brake();        
        }
    };
    chassis.moveToPoint(14, -16, 750, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task3{[=] {
        pros::delay(850);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(32, 5, 1000, {.forwards = false, .maxSpeed = 127}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    /*chassis.turnToPoint(-32, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-32, 26, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(100);
    chassis.moveToPoint(-32, 10, 1000, {.forwards = false, .maxSpeed = 127}, false);*/
    chassis.turnToPoint(51, 20, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(51, 20, 1000, {.forwards = true, .maxSpeed = 60}, false);
    //chassis.turnToPoint(42, 28, 1000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(54, 34, 1000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(34, 16, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(34, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(34, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-6, 54, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-6, 54, 1750, {.forwards = true, .maxSpeed = 90}, false);
    pros::delay(250);
    pros::Task task4{[=] {
        pros::delay(750);
        intake.brake();        
        }
    };
    chassis.moveToPoint(44, 0, 1500, {.forwards = false, .maxSpeed = 127}, false);
    nextState();
    //chassis.moveToPoint(8, -12, 1000, {.forwards = false, .maxSpeed = 127}, false);
}

void RedLeftSigWP() {
    chassis.setPose(0, -8, 0);
    //chassis.moveToPoint(0, -9, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-10, -14, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(8, -14, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(900);
        intake.brake();        
        }
    };
    chassis.moveToPoint(14, -16, 750, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task3{[=] {
        pros::delay(850);
        backClaw.set_value(true);       
        }
    };
    chassis.moveToPoint(32, 5, 1000, {.forwards = false, .maxSpeed = 90}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    chassis.turnToPoint(36, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(36, 30, 1000, {.forwards = true, .maxSpeed = 90}, false);
    pros::delay(250);
    chassis.turnToPoint(12, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(12, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    chassis.moveToPoint(20, -20, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    pros::Task task4{[=] {
        pros::delay(850);
        backClaw.set_value(true);       
        }
    };
    chassis.moveToPoint(42, -28, 1000, {.forwards = false, .maxSpeed = 90}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    chassis.turnToPoint(46, -54, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(46, -54, 1000, {.forwards = true, .maxSpeed = 90}, false);
    pros::delay(750);
    intake.brake();
    chassis.moveToPoint(58, -20, 1500, {.forwards = false, .maxSpeed = 127}, false);
    descoreState();
}

void BlueLeftRush() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    intake.move_voltage(12000);
    pros::Task task1{[=] {
        stick.set_value(true);        
        pros::delay(800);
        stickClaw.set_value(true);
    }
    };
    chassis.moveToPoint(0, 40, 900, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(650);
        intake.brake();
    }
    };
    chassis.moveToPoint(0, 16, 1000, {.forwards = false, .maxSpeed = 127}, false);
    stickClaw.set_value(false);
    pros::delay(250);
    stick.set_value(false);
    pros::delay(250);
    chassis.turnToPoint(22, 34, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task3{[=] {
        pros::delay(1000);
        backClaw.set_value(true);
        pros::delay(250);
        intake.move_voltage(12000);
    }
    };
    chassis.moveToPoint(22, 34, 2000, {.forwards = false, .maxSpeed = 45}, false);
    pros::delay(500);
    intake.brake();
    backClaw.set_value(false);
    chassis.moveToPoint(0, 20, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-4, 36, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task4{[=] {
        pros::delay(900);
        backClaw.set_value(true);
    }
    };
    chassis.moveToPoint(-4, 36, 1000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(250);
    pros::Task task5{[=] {
        pros::delay(1000);
        intake.move_voltage(12000);
    }
    };
    chassis.moveToPoint(-1, -15, 1750, {.forwards = true, .maxSpeed = 60} ,false);
    chassis.turnToPoint(22, 5, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task6{[=] {
        pros::delay(800);
        intake.brake();
    }
    };
    chassis.moveToPoint(22, 5, 1000, {.forwards = true, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(250);
    chassis.turnToPoint(18, 32, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task7{[=] {
        pros::delay(900);
        backClaw.set_value(true);
        }
    };
    chassis.moveToPoint(18, 32, 1000, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
}

void BlueRightSimpleWP() {
    chassis.setPose(0, -8, 0);
    //chassis.moveToPoint(0, -9, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(10, -14, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(-8, -16, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(900);
        intake.brake();        
        }
    };
    chassis.moveToPoint(-16, -14, 750, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task3{[=] {
        pros::delay(850);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-31, 3, 1000, {.forwards = false, .maxSpeed = 127}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    /*chassis.turnToPoint(-32, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-32, 26, 1000, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(100);
    chassis.moveToPoint(-32, 10, 1000, {.forwards = false, .maxSpeed = 127}, false);*/
    chassis.turnToPoint(-52, 16, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-52, 16, 1000, {.forwards = true, .maxSpeed = 60}, false);
    //chassis.turnToPoint(42, 28, 1000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-56, 36, 1500, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(-36, 16, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-36, 34, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-36, 34, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(8, 58, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(8, 58, 1750, {.forwards = true, .maxSpeed = 90}, false);
    pros::delay(250);
    pros::Task task4{[=] {
        pros::delay(900);
        intake.brake();        
        }
    };
    chassis.moveToPoint(-48, -2, 1500, {.forwards = false, .maxSpeed = 60}, false);
    nextState();
    //chassis.moveToPoint(8, -12, 1000, {.forwards = false, .maxSpeed = 127}, false);
}

void BlueRightSigWP() {
    chassis.setPose(0, -8, 0);
    //chassis.moveToPoint(0, -9, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(10, -14, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::Task task1{[=] {
        pros::delay(100);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(-8, -16, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(900);
        intake.brake();        
        }
    };
    chassis.moveToPoint(-16, -14, 750, {.forwards = true, .maxSpeed = 45}, false);
    pros::Task task3{[=] {
        pros::delay(850);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-30, 3, 1250, {.forwards = false, .maxSpeed = 127}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    chassis.turnToPoint(-36, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-36, 30, 1000, {.forwards = true, .maxSpeed = 90}, false);
    chassis.turnToPoint(-12, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-12, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    chassis.moveToPoint(-20, -20, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    pros::Task task4{[=] {
        pros::delay(1000);
        backClaw.set_value(true);       
        }
    };
    chassis.moveToPoint(-42, -34, 1000, {.forwards = false, .maxSpeed = 90}, false);
    //backClaw.set_value(true);
    pros::delay(250);
    chassis.turnToPoint(-46, -56, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-46, -56, 1000, {.forwards = true, .maxSpeed = 90}, false);
    pros::delay(750);
    intake.brake();
    chassis.moveToPoint(-60, -24, 1500, {.forwards = false, .maxSpeed = 127}, false);
    descoreState();
}

void Skills() {
    // skills part one
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, -4, 0);
    // run intake to score preload on alliance stake
    intake.move_voltage(12000);
    pros::delay(250);
    intake.move_voltage(-12000);
    pros::delay(250);
    intake.brake();
    //Cake.set_value(true);
    // move to mobile goal on left
    chassis.moveToPoint(0, 8, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-24, 8, 750, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(700);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-24, 8, 750, {.forwards = false, .maxSpeed = 60}, false);
    // move to intake red ring
    chassis.turnToPoint(-24, 30, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task2{[=] {
        pros::delay(150);
        intake.move_voltage(12000);        
        }
    };
    chassis.moveToPoint(-24, 30, 750, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(-46, 50, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-46, 50, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-48, 78, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-48, 78, 1000, {.forwards = true, .maxSpeed = 127}, false);
    target = 3200;
    // move to intake red ring near wall stake
    chassis.moveToPoint(-38, 62, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task3{[=] {
        pros::delay(500);
        intake.move_voltage(-12000); 
        pros::delay(100);
        intakeState();       
        }
    };
    chassis.turnToPoint(-60, 62, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-60, 62, 1000, {.forwards = true, .maxSpeed = 60}, false);
    descoreState();
    pros::delay(750);
    descoreState();
    intakeState();
    intakeState();
    pros::delay(250);
    //chassis.turnToPoint(-44, 60, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-45, 64, 1000, {.forwards = false, .maxSpeed = 90}, false);
    chassis.turnToPoint(-45, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //hang.set_value(true);
    chassis.moveToPoint(-45, 0, 2500, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(1000);
    chassis.turnToPoint(-58, 10,750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-58, 10, 750, {.forwards = true, .maxSpeed = 127}, false);
    // drop mobile goal in corner
    chassis.turnToPoint(-64, 0, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-64, 0, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    backClaw.set_value(false);
    intakeState();
    pros::delay(250);
    // skills part two
    intake.move_voltage(12000);
    chassis.moveToPoint(-48, 108, 2500, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task4{[=] {
        pros::delay(400);
        intake.brake();      
        }
    };
    chassis.turnToPoint(-26, 122, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task5{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(500);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-26, 122, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-58, 132, 1000, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(-58, 140, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-28, 122, 1500, {.forwards = true, .maxSpeed = 90}, false);
    /*
    //distance sensor code
    chassis.turnToHeading(280, 1500, {.maxSpeed = 127}, false);
    int count1 = 0;
    pros::Task task5{[=] {
        pros::delay(600);
        backClaw.set_value(true);      
        }
    };
    while (distance_sensor.get() < 1750 && count1 < 75) { 
        left_motors.move_voltage(((distance_sensor.get())-1750)*20);
        right_motors.move_voltage(((distance_sensor.get())-1750)*20);
        pros::delay(20);
        count1++;
    }*/
    chassis.turnToPoint(0, 122, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task6{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(650);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(0, 122, 1500, {.forwards = false, .maxSpeed = 90}, false);
    chassis.turnToPoint(-24, 98, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-24, 98, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(2, 72, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    chassis.moveToPoint(2, 72, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(100);
    intake.move_voltage(12000);
    pros::delay(250);
    intake.brake();
    pros::Task task7{[=] {
        pros::delay(800); 
        intake.move_voltage(12000);  
        }
    };
    chassis.moveToPoint(46, 25, 2500, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(46, 2, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(46, 2, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(64, 0, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(64, 0, 1000, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(-12000);
    backClaw.set_value(false);
    pros::delay(250);
    intake.move_voltage(12000);
    chassis.moveToPoint(62, 10, 1000, {.forwards = true, .maxSpeed = 127}, false);
    target = 3200;
    pros::delay(250);
    chassis.turnToPoint(22, 10, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task9{[=] {
        pros::delay(1000); 
        backClaw.set_value(true);
        }
    };
    chassis.moveToPoint(22, 10, 1500, {.forwards = false, .maxSpeed = 60}, false);
    /*
    UNCOMMENT THIS TASK WHEN YOU CONTINUE TO SKILLS PART FOUR
    pros::Task task10{[=] {
        pros::delay(500);
        intake.move_voltage(-12000); 
        pros::delay(100);
        intakeState();       
        }
    };*/

    /*//skills part four
    chassis.setPose(0, 0, 0);
    //backClaw.set_value(true);
    chassis.moveToPoint(0, -10, 1500, {.forwards = false, .maxSpeed = 90}, false);
    //descoreState();
    //intakeState();
    intake.move_voltage(12000);
    chassis.turnToPoint(-20, -30, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, -30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);
    chassis.turnToPoint(-46, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-46, -14, 1000, {.forwards = true, .maxSpeed = 90}, false);
    chassis.turnToPoint(-38, -10, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-38, -10, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-52, -4, 1000, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(100);
    chassis.moveToPoint(-52, -4, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-26, -28, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(0, -60, 1000, {.forwards = false, .maxSpeed = 127}, false);
    descoreState();
    intake.brake();
    chassis.moveToPoint(0, -60, 2000, {.forwards = false, .maxSpeed = 60}, false);
    left_motors.move_voltage(2400);
    right_motors.move_voltage(2400);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();*/
} 

void PidTuning() {
    // set position to x:0, y:0, heading:0
    /*chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);*/

    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // move 48" forwards
    chassis.moveToPoint(0, 24, 10000);
}

/*void WallStakeMacro() ;
    while (true) {
        pros::Task task{[=] {
            if (distance_sensor.get() < 10) {
                pros::delay(50);
                intake.move_voltage(-12000);
            }
        }
        };
        intake.move_voltage(12000);
    }
}*/


void Test() {
    /*int k = 0;
    intake.move_voltage(12000);
    k = 1;
    while (k == 1) {
        if (distance_sensor.get() <= 18) {
            while (distance_sensor.get() <= 18) {
                pros::delay(100); 
                intake.move_voltage(-12000);
                pros::delay(1000);
            }
            k--;
        }
        else {
            intake.move_voltage(120000);
        }
    }
    intake.move_voltage(12000);*/

    // red rings
    backClaw.set_value(true);
    pros::delay(250);
    intake.move_voltage(12000);
    optical_sensor.set_led_pwm(100);
    while (true) {
        if (optical_sensor.get_hue() < 240 && optical_sensor.get_hue() > 210) {
            while (optical_sensor.get_hue() < 240 && optical_sensor.get_hue() > 210) {
                intake.move_voltage(12000);
            }
            pros::delay(50);
            intake.move_voltage(-12000);
            pros::delay(250);
            intake.move_voltage(120000);
        }
        else {
            intake.move_voltage(12000);
        }
    }

    //distance sensor code
    while (distance_sensor.get() < 1400) { 
        left_motors.move_voltage((1400-distance_sensor.get())*24);
        right_motors.move_voltage((1400-distance_sensor.get())*24);
        pros::delay(20);
    }

    // blue rings
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    optical_sensor.set_led_pwm(75);
    // print position to brain screen
    pros::Task initialize([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // print measurements from the rotation sensor
            pros::lcd::print(3, "Rot: %i", rotation_sensor.get_position());
            pros::lcd::print(4, "Tracking Pod: %i", tracking_pod.get_position());
            // print measurements from the distance sensors
            pros::lcd::print(5, "distanceSensor: %f", fmt::to_string((distance_sensor.get())));
            pros::lcd::print(6, "opticalSensor: %lf", optical_sensor.get_hue());
            pros::lcd::print(7, "lineSensor: %i", line_sensor.get_value());
            pros::lcd::print(8, "Target: %i", target);
            pros::lcd::print(9, "Error: %i", error);

            liftControl();

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
    pros::Task run{[&]{
        while (true) {

            //redColorSort();

            //blueColorSort();
            
            //unjam();

            pros::delay(50);
        }
    }
    };
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
    bool stickUp = false;
    bool stickDown = true;
    bool stickClawOpened = false;
    bool stickClawClosed = true;
    bool tipUp = false;
    bool tipDown = true;
    target = 5;

    // loop forever

    while (true) {
        //Tank drive
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        //Arcade drive
        // get left y and right x positions
        /*int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);*/
    
        //while (intakeStop == false) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(12000);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(-12000);
            }
            else {
                intake.move_voltage(0);
            }
        //}

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

        /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            WallStakeMacro();
        }*/

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) and stickClawOpened) {
            if (stickClawClosed) {
                stickClaw.set_value(false);
                stickClawClosed = false;
            }
            else if (not stickClawClosed and stickClawOpened){
                stickClaw.set_value(true);
                stickClawClosed = true;
            }
            stickClawOpened = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            stickClawOpened = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) and stickUp) {
            if (stickDown) {
                stick.set_value(false);
                stickDown = false;
            }
            else if (not stickDown and stickUp){
                stick.set_value(true);
                stickDown = true;
            }
            stickUp = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            stickUp = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) and tipUp) {
            if (tipDown) {
                tip.set_value(false);
                tipDown = false;
            }
            else if (not tipDown and tipUp){
                tip.set_value(true);
                tipDown = true;
            }
            tipUp = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            tipUp = true;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            nextState();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intakeState();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            descoreState();
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