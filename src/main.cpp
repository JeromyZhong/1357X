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
pros::Distance back_distance_sensor(3); //Back


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


pros::MotorGroup intake({1, -10}, pros::MotorGearset::green);
pros::adi::DigitalOut longGoal(8);
pros::adi::DigitalOut middleGoal(7);
pros::adi::DigitalOut matchloader(6);
pros::adi::DigitalOut wing(5);


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


void Skills() {
    wing.set_value(true);
    chassis.setPose(0, 0, 0);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 30, 1000,{.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    intake.move_voltage(12000);        
    chassis.turnToPoint(-26, 28, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-16, 28, 750, {.forwards = true, .maxSpeed = 90}, false);
    chassis.moveToPoint(-20, 28, 1500, {.forwards = true, .maxSpeed = 30}, false);
    chassis.moveToPoint(14, 38, 1000, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    chassis.moveToPoint(76, 43, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(92, 31, 1000, {.forwards = false, .maxSpeed = 127}, false);
    matchloader.set_value(false);
    //chassis.moveToPoint(82, 42, 2000, {.forwards = false, .maxSpeed = 127}, false);
    /*chassis.turnToHeading(180, 1000);
    pros::delay(2500);
    while (back_distance_sensor.get() < 850) { // Greater than 850 works
        left_motors.move_voltage((850-back_distance_sensor.get())*24);
        right_motors.move_voltage((850-back_distance_sensor.get())*24);
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();
    pros::delay(15000);*/
    //chassis.moveToPoint(82, 31, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(72, 31, 1000, {.forwards = false, .maxSpeed = 127}, false);
    longGoal.set_value(false);
    chassis.moveToPoint(72, 31, 1500, {.forwards = false, .maxSpeed = 60}, false);
    intake.move_voltage(12000);
    pros::delay(2000);
    matchloader.set_value(true);
    longGoal.set_value(true);
    chassis.turnToPoint(106, 31, 500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(106, 31, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(110, 31, 1500, {.forwards = true, .maxSpeed = 30}, false);
    intake.brake();
    longGoal.set_value(false);
    chassis.moveToPoint(72, 31, 1500, {.forwards = false, .maxSpeed = 60}, false);
    intake.move_voltage(12000);
    pros::delay(2500);
    matchloader.set_value(false);
    chassis.moveToPoint(114, 14, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(118, 0, 1000, {.forwards = true, .maxSpeed = 100}, false);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    chassis.moveToPoint(118, 0, 1500, {.forwards = true, .maxSpeed = 100}, false);
    matchloader.set_value(true);
    pros::delay(250);
    chassis.moveToPoint(118, -32, 1500, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(250);
    chassis.moveToPoint(118, -60, 1500, {.forwards = true, .maxSpeed = 100}, false);
    chassis.moveToPoint(94, -85, 1500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(94, -150, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(94, -150, 1500, {.forwards = false, .maxSpeed = 60}, false);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 17, 1000, {.forwards = true, .maxSpeed = 127}, false);
    intake.brake();
    longGoal.set_value(false);
    chassis.turnToPoint(-12, 18, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-12, 18, 1000, {.forwards = false, .maxSpeed = 60}, false);
    left_motors.move_voltage(-6000);
    right_motors.move_voltage(-6000);
    pros::delay(500);
    left_motors.brake();
    right_motors.brake();
    chassis.setPose(0, 0, 0);
    intake.move_voltage(12000);
    pros::delay(2000);
    //matchloader.set_value(true);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 32, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(0, 36, 1500, {.forwards = true, .maxSpeed = 30}, false);
    intake.brake();
    longGoal.set_value(false);
    chassis.moveToPoint(0, 0, 1500, {.forwards = false, .maxSpeed = 60}, false);
    intake.move_voltage(12000);
    pros::delay(2500);
    matchloader.set_value(false);
    intake.brake();
    chassis.moveToPoint(0, 30, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(11, 5, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(15, -64, 2000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-1, -76, 1000, {.forwards = false, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    chassis.turnToPoint(1, -96, 1000, {.forwards = true, .maxSpeed = 127}, false);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    chassis.moveToPoint(1, -96, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(1, -100, 1500, {.forwards = true, .maxSpeed = 30}, false);
    intake.brake();
    longGoal.set_value(false);
    chassis.moveToPoint(1, -45, 1500, {.forwards = false, .maxSpeed = 60}, false);
    matchloader.set_value(false);
    intake.move_voltage(12000);
    pros::delay(2500);
    chassis.moveToPoint(-22, -96, 1000, {.forwards = true, .maxSpeed = 127}, false);
    //chassis.turnToPoint(-34, -104, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-34, -104, 1000, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(250);
    matchloader.set_value(true);
    longGoal.set_value(true);
    chassis.moveToPoint(-59, -108, 2000, {.forwards = true, .maxSpeed = 127}, false);


}


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
    wing.set_value(true);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 30, 1000,{.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    intake.move_voltage(12000);        
    chassis.turnToPoint(-26, 28, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-16, 28, 750, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(250);
    intake.brake();
    chassis.moveToPoint(18, 28, 750, {.forwards = false, .maxSpeed = 127}, false);
    longGoal.set_value(false);
    intake.brake();
    chassis.moveToPoint(24, 28, 750, {.forwards = false, .maxSpeed = 30}, false);
    intake.move_voltage(12000);
    pros::delay(1500);
    intake.brake();
    matchloader.set_value(false);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    chassis.moveToPoint(22, 18, 1000, {.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    chassis.moveToPoint(26, 8, 1000, {.forwards = true, .maxSpeed = 60}, false);
    chassis.turnToPoint(39, -10, 1000, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    middleGoal.set_value(true);
    chassis.moveToPoint(39, -10, 1250, {.forwards = false, .maxSpeed = 45}, false);
    intake.move_voltage(12000);
    pros::delay(1500);
    matchloader.set_value(false);
    chassis.moveToPoint(20, 19, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(40, 23, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(40, 23, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(66, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
}

void rightAuto() {
    chassis.setPose(0, 0, 0);
    wing.set_value(true);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 30, 1000,{.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    intake.move_voltage(12000);        
    chassis.turnToPoint(26, 34, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(16, 34, 750, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(250);
    intake.brake();
    chassis.moveToPoint(-18, 34, 750, {.forwards = false, .maxSpeed = 127}, false);
    longGoal.set_value(false);
    intake.brake();
    chassis.moveToPoint(-24, 34, 750, {.forwards = false, .maxSpeed = 30}, false);
    intake.move_voltage(12000);
    pros::delay(1500);
    intake.brake();
    matchloader.set_value(false);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    chassis.moveToPoint(-4, 34, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-20, 12, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-20, 12, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-36, -4, 2000, {.forwards = true, .maxSpeed = 30}, false);
    intake.move_voltage(-12000);
    pros::delay(1000);
    intake.brake();
    chassis.moveToPoint(-10, 22, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-34, 22, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-34, 22, 1000, {.forwards = false, .maxSpeed = 127}, false);
}


void soloWP() {
    chassis.setPose(0, 0, 0);
    wing.set_value(true);
    longGoal.set_value(true);
    chassis.moveToPoint(0, 28, 750,{.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(true);
    intake.move_voltage(12000);        
    chassis.turnToPoint(26, 32, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(16, 32, 750, {.forwards = true, .maxSpeed = 100}, false);
    pros::delay(200);
    intake.brake();
    chassis.moveToPoint(-20, 31, 750, {.forwards = false, .maxSpeed = 127}, false);
    longGoal.set_value(false);
    intake.brake();
    intake.move_voltage(12000);
    pros::delay(1100);
    intake.brake();
    matchloader.set_value(false);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    chassis.moveToPoint(-10, 32, 500, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-24, 4, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::Task task1{[=] {
        pros::delay(550);
        matchloader.set_value(true);  
        }
    };
    chassis.moveToPoint(-24, 4, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-22, -34, 500, {.forwards = true, .maxSpeed = 127}, false);
    matchloader.set_value(false);
    pros::Task task2{[=] {
        pros::delay(650);
        matchloader.set_value(true);  
        }
    };
    chassis.moveToPoint(-22, -34, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-35, -22, 500, {.forwards = false, .maxSpeed = 127}, false);
    intake.brake();
    middleGoal.set_value(true);
    chassis.moveToPoint(-35, -22, 750, {.forwards = false, .maxSpeed = 127}, false);
    intake.move_voltage(12000);
    pros::delay(1500);
    chassis.moveToPoint(4, -59, 1000, {.forwards = true, .maxSpeed = 127}, false);
    middleGoal.set_value(false);
    chassis.turnToPoint(20, -59, 750, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(20, -59, 750, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(200);
    intake.brake();
    chassis.moveToPoint(-18, -59, 1000, {.forwards = false, .maxSpeed = 90}, false);
    longGoal.set_value(false);
    intake.move_voltage(12000);
}

void sevenBallLeft() {
    chassis.setPose(0, 0, 0);
    wing.set_value(true);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    pros::Task task1{[=] {
        pros::delay(400);
        matchloader.set_value(true);  
        }
    };
    chassis.moveToPoint(-8, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-31, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-31, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-31, -24, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(-31, -24, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);
    intake.brake();
    chassis.moveToPoint(-31, 25, 1000, {.forwards = false, .maxSpeed = 60}, false);
    longGoal.set_value(false);
    intake.move_voltage(12000);
    pros::delay(3000);
    intake.brake();
    matchloader.set_value(false);
    chassis.moveToPoint(-32, -2, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(-44, 20, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(-44, 20, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(-42, 44, 1000, {.forwards = false, .maxSpeed = 127}, false);
    wing.set_value(false);
    chassis.moveToPoint(-42, 44, 2000, {.forwards = false, .maxSpeed = 127}, false);
}

void sevenBallRight() {
    chassis.setPose(0, 0, 0);
    wing.set_value(true);
    longGoal.set_value(true);
    intake.move_voltage(12000);
    pros::Task task1{[=] {
        pros::delay(400);
        matchloader.set_value(true);  
        }
    };
    chassis.moveToPoint(8, 26, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(32, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(32, 0, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(32, -24, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.moveToPoint(32, -24, 1000, {.forwards = true, .maxSpeed = 127}, false);
    pros::delay(250);
    intake.brake();
    chassis.moveToPoint(32, 25, 1000, {.forwards = false, .maxSpeed = 60}, false);
    longGoal.set_value(false);
    intake.move_voltage(12000);
    pros::delay(3000);
    intake.brake();
    matchloader.set_value(false);
    chassis.moveToPoint(32, -2, 1000, {.forwards = true, .maxSpeed = 127}, false);
    chassis.turnToPoint(21, 20, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.moveToPoint(21, 20, 1000, {.forwards = false, .maxSpeed = 127}, false);
    chassis.turnToPoint(21, 42, 1000, {.forwards = false, .maxSpeed = 127}, false);
    wing.set_value(false);
    chassis.moveToPoint(21, 42, 2000, {.forwards = false, .maxSpeed = 127}, false);
}


void wearebuns() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 4, 1000, {.forwards = true, .maxSpeed = 127}, false);
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


    while (back_distance_sensor.get() < 460){ //Go to matchload bar code
        left_motors.move_voltage((460-back_distance_sensor.get())*24);
        right_motors.move_voltage((460-back_distance_sensor.get())*24);
        pros::delay(20);
    }
    left_motors.brake();
    right_motors.brake();
}


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.calibrate(); // calibrate sensors
    //optical_sensor.set_led_pwm(75);
    // print position to brain screen
    pros::Task initialize([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Distance Sensor: %f", back_distance_sensor.get()); //
            // print measurements from the rotation sensor
            //pros::lcd::print(3, "Rot: %i", rotation_sensor.get_position());
            //pros::lcd::print(4, "Tracking Pod: %i", tracking_pod.get_position());
            // print measurements from the distance sensors
            //pros::lcd::print(5, "distanceSensor: %f", fmt::to_string((distance_sensor.get())));
            //pros::lcd::print(6, "opticalSensor: %lf", optical_sensor.get_hue());
            //pros::lcd::print(7, "lineSensor: %i", line_sensor.get_value());
            //pros::lcd::print(8, "Target: %i", target);
            //pros::lcd::print(9, "Error: %i", error);
            //controller.clear_line(0);
            //controller.print(0, 0, "Distance: %.2f", back_distance_sensor.get());




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
    /*pros::Task run{[&]{
        while (true) {


            //redColorSort();
           
            //unjam();


            pros::delay(50);
        }
    }
    };*/
    sevenBallRight();
    longGoal.set_value(false);
    middleGoal.set_value(false);
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
    wing.set_value(true);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool longGoalOpen = true;
    bool block = false;
    bool middleGoalUp = false;
    bool middleGoalDown = true;
    bool matchloaderDown = false;
    bool matchloaderUp = true;
    bool wingDown = true;
    bool wingUp = false;


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


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) and wingDown) {
            if (wingUp) {
                wing.set_value(false);
                wingUp = false;
            }
            else if (not wingUp and wingDown){
                wing.set_value(true);
                wingUp = true;
            }
            wingDown = false;
        }


        if (not controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            wingDown = true;
        }


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            middleGoal.set_value(true);
            longGoal.set_value(true);
            longGoalOpen = false;
            block = true;
            middleGoalUp = true;
            middleGoalDown = false;
        }


        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            middleGoal.set_value(false);
            longGoal.set_value(false);
            longGoalOpen = true;
            block = false;
            middleGoalUp = false;
            middleGoalDown = true;
        }


         // delay to save resources
        pros::delay(25);


    }
}