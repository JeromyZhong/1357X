
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
pros::MotorGroup left_motors({-11, -12, -13}, pros::MotorGearset::green); // left motors on ports 11, 12, 13; all reverse
pros::MotorGroup right_motors({20, 19, 18}, pros::MotorGearset::green); // right motors on ports 18, 19, 20; all forwards

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
pros::Imu inertial_sensor(9);

//Distance Sensor
//pros::Distance distance_sensor(6); //Right
//pros::Distance back_distance_sensor(7); //Back

//Optical Sensor
//pros::Optical optical_sensor(10);

// Rotation Sensor
//pros::Rotation rotation_sensor(7);
//pros::Rotation tracking_pod(5);

// Line Tracker
//pros::adi::AnalogIn line_sensor('A');

//Horizontal Tracking Wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::OLD_275, 0);

//Vertical Tracking Wheel
//lemlib::TrackingWheel vertical_tracking_wheel(&tracking_pod, lemlib::Omniwheel::NEW_2, 0);

//Final Configuration of sensors
lemlib::OdomSensors sensors(
	nullptr, // vertical tracking wheel 1
    nullptr,
    nullptr, // horizontal tracking wheel 1
    nullptr,
    &inertial_sensor // inertial sensor
);

// PID Constants
// Lateral PID Controller
lemlib::ControllerSettings lateral_controller(
    6.75, // proportional gain (kP)
    0, // integral gain (kI)
    18.5, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
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
    2.3, // proportional gain (kP)
    0, // integral gain (kI)
    14, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in degrees
    0, // small error range timeout, in milliseconds
    0, // large error range, in degrees
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
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

pros::MotorGroup intake({1, -2}, pros::MotorGearset::green);
pros::adi::DigitalOut longGoal(8);
pros::adi::DigitalOut middleGoal(7);
pros::adi::DigitalOut matchloader(6);

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

/*void lineTrack() {
    while (true) {
        int sensorValue = line_sensor.get_value();

    }
}*/

/*void Skills() {
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
    chassis.moveToPoint(0, 9, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-24, 9, 750, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(650);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-24, 9, 750, {.forwards = false, .maxSpeed = 60}, false);
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
    chassis.turnToPoint(-48, 84, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-48, 84, 1000, {.forwards = true, .maxSpeed = 127}, false);
    target = 3200;
    // move to intake red ring near wall stake
    chassis.moveToPoint(-38, 62, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task3{[=] {
        pros::delay(500);
        intake.move_voltage(-12000);
        pros::delay(25);
        intakeState();       
        }
    };
    chassis.turnToPoint(-60, 62, 750, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-60, 62, 1000, {.forwards = true, .maxSpeed = 60}, false);
    descoreState();
    pros::delay(500);
    descoreState();
    intakeState();
    intakeState();
    pros::delay(250);
    //chassis.turnToPoint(-44, 60, 1500, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-45, 64, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-45, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //hang.set_value(true);
    chassis.moveToPoint(-45, 0, 2500, {.forwards = true, .maxSpeed = 45}, false);
    pros::delay(750);
    chassis.turnToPoint(-58, 10,750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-58, 10, 750, {.forwards = true, .maxSpeed = 127}, false);
    // drop mobile goal in corner
    chassis.turnToPoint(-64, 2, 750, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-64, 2, 750, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    intakeState();
    pros::delay(250);
    // skills part two
    intake.move_voltage(12000);
    chassis.moveToPoint(-48, 110, 2500, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task4{[=] {
        pros::delay(250);
        intake.brake();      
        }
    };
    chassis.turnToPoint(-22, 124, 1000, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task5{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(600);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(-22, 124, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-58, 132, 1000, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(-58, 140, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-24, 118, 1500, {.forwards = true, .maxSpeed = 90}, false);
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
    /*chassis.turnToPoint(2, 119, 1500, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task6{[=] {
        // pick up mobile goal while robot is still moving
        pros::delay(800);
        backClaw.set_value(true);        
        }
    };
    chassis.moveToPoint(2, 119, 1500, {.forwards = false, .maxSpeed = 60}, false);
    chassis.turnToPoint(-22, 98, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(-22, 98, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(0, 72, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task7{[=] {
        intake.brake();
        pros::delay(750); 
        intake.move_voltage(12000); 
        pros::delay(500);
        intake.brake();
        pros::delay(1000);
        intake.move_voltage(12000); 
        }
    };
    chassis.moveToPoint(0, 68, 2000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.moveToPoint(46, 20, 2500, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(48, 2, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(48, 2, 750, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(58, -1, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(58, -1, 750, {.forwards = false, .maxSpeed = 127}, false);
    backClaw.set_value(false);
    pros::delay(250);
    intake.move_voltage(12000);
    chassis.moveToPoint(64, 7, 1000, {.forwards = true, .maxSpeed = 127}, false);
    target = 3200;
    chassis.turnToPoint(22, 7, 750, {.forwards = false, .maxSpeed = 127}, false);
    pros::Task task9{[=] {
        pros::delay(950); 
        backClaw.set_value(true);
        }
    };
    chassis.moveToPoint(22, 7, 1500, {.forwards = false, .maxSpeed = 60}, false);
    pros::Task task10{[=] {
        pros::delay(500);
        intake.move_voltage(-12000);
        pros::delay(25);
        intakeState();       
        }
    };
    chassis.turnToPoint(50, 30, 750, {.forwards = true, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    chassis.moveToPoint(50, 30, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(62, 53, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(62, 53, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(74, 55, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task11{[=] {
        pros::delay(1000);
        intake.brake();  
        }
    };
    chassis.moveToPoint(74, 55, 1000, {.forwards = true, .maxSpeed = 60}, false);
    descoreState();
    pros::delay(500);
    descoreState();
    intakeState();
    intakeState();

    //skills part four
    //chassis.setPose(0, 0, 0);
    //backClaw.set_value(true);
    chassis.moveToPoint(53, 50, 1000, {.forwards = false, .maxSpeed = 127}, false);
    //descoreState();
    //intakeState();
    intake.move_voltage(12000);
    /*chassis.turnToPoint(-20, -30, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, -30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, -14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);*/
    /*chassis.turnToPoint(53, 118, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(53, 100, 1250, {.forwards = true, .maxSpeed = 90}, false);
    chassis.moveToPoint(54, 118, 1000, {.forwards = true, .maxSpeed = 45}, false);
    chassis.turnToPoint(68, 100, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(68, 100, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(75, 120, 750, {.forwards = false, .maxSpeed = 127}, false);
    left_motors.move_voltage(6000);
    right_motors.move_voltage(6000);
    pros::delay(300);
    left_motors.brake();
    right_motors.brake();
    backClaw.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(75, 124, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(42, 100, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(0, 70, 750, {.forwards = false, .maxSpeed = 127}, false);
    descoreState();
    intake.brake();
    chassis.moveToPoint(0, 70, 1000, {.forwards = false, .maxSpeed = 60}, false);
    left_motors.move_voltage(3000);
    right_motors.move_voltage(3000);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();
}*/ 

void PidTuning() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);

    // set position to x:0, y:0, heading:0
    /*chassis.setPose(0, 0, 0);
    // move 48" forwards
    chassis.moveToPoint(0, 24, 10000);*/
}

void leftAuto() {
    chassis.setPose(0, 0, 0);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 30, 1000,{.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        pros::delay(10);
        intake.move_voltage(12000);
        matchloader.set_value(true);        
        }
    };

    chassis.moveToPoint(-12, 30, 1000, {.forwards = true, .maxSpeed = 60}, false);
    pros::delay(250);
    intake.brake();
    chassis.moveToPoint(16, 30, 1000, {.forwards = false, .maxSpeed = 90}, false);
    longGoal.set_value(false);
    intake.brake();
    chassis.moveToPoint(24, 30, 1000, {.forwards = false, .maxSpeed = 30}, false);
    intake.move_voltage(12000);
    pros::delay(2000);
    matchloader.set_value(false);
    longGoal.set_value(true);
    chassis.moveToPoint(24, 18, 1260, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(30, 4, 750, {.forwards = true, .maxSpeed = 45}, false);
    chassis.turnToPoint(40, -6, 1000, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    middleGoal.set_value(true);
    chassis.moveToPoint(40, -6, 750, {.forwards = false, .maxSpeed = 30}, false);
    intake.move_voltage(12000);
    pros::delay(750);

}

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

    //distance sensor code
    /*while (distance_sensor.get() < 1400) { 
        left_motors.move_voltage((1400-distance_sensor.get())*24);
        right_motors.move_voltage((1400-distance_sensor.get())*24);
        pros::delay(20);
    }*/
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    //optical_sensor.set_led_pwm(75);
    // print position to brain screen
    pros::Task initialize([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // print measurements from the rotation sensor
            //pros::lcd::print(3, "Rot: %i", rotation_sensor.get_position());
            //pros::lcd::print(4, "Tracking Pod: %i", tracking_pod.get_position());
            // print measurements from the distance sensors
            //pros::lcd::print(5, "distanceSensor: %f", fmt::to_string((distance_sensor.get())));
            //pros::lcd::print(6, "opticalSensor: %lf", optical_sensor.get_hue());
            //pros::lcd::print(7, "lineSensor: %i", line_sensor.get_value());
            //pros::lcd::print(8, "Target: %i", target);
            //pros::lcd::print(9, "Error: %i", error);

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
            
            //unjam();

            pros::delay(50);
        }
    }
    };
    leftAuto();
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
    bool longGoalOpen = true;
    bool block = false;
    bool middleGoalUp = false;
    bool middleGoalDown = true;
    bool matchloaderDown = false;
    bool matchloaderUp = true;

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
    
        //while (intakeStop == false) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(-12000);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(12000);
            }
            else {
                intake.move_voltage(0);
            }
        //}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) and matchloaderUp) {
            if (matchloaderDown) {
                matchloader.set_value(false);
                matchloaderDown = false;
            }
            else if (not matchloaderDown and matchloaderUp){
                matchloader.set_value(true);
                matchloaderDown = true;
            }
            matchloaderUp = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            matchloaderUp = true;
        }


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and longGoalOpen) {
            if (block) {
                longGoal.set_value(false);
                block = false;
            }
            else if (not block and longGoalOpen){
                longGoal.set_value(true);
                block = true;
            }
            longGoalOpen = false;
        }

        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            longGoalOpen = true;
        }

        /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) {
            middleGoal.set_value(true);
            longGoal.set_value(true);
            longGoalOpen = false;
            block = true;
            middleGoalUp = true;
            middleGoalDown = false;
        }
        }*/

        /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) {
            middleGoal.set_value(false);
            longGoal.set_value(false);
            longGoalOpen = true;
            block = false;
            middleGoalUp = false;
            middleGoalDown = true;
        }
        }*/

         // delay to save resources
        pros::delay(25);

    }
}