#include "main.h"
#include "globals.hpp"

double circumference = pi*4.125;
double rotations = 24/circumference;
double ticks = rotations*360;

void my_autonomous() {
    // drive::turn_radians(pi*1.5);
    // pros::delay(1000);
    // drive::turn_radians(-pi/2);
    // pros::delay(1000);
    // drive::turn_global(0);
    drive::turn_radians(pi/2);
}

void right_auton() {
    profileController->generatePath({{0_in, 0_in, 0_deg}, {40_in, 0_in, 0_deg}}, "A");
    profileController->setTarget("A");
    jawcontroller->setTarget(-400);
    profileController->waitUntilSettled();
    jawcontroller->setTarget(-500);
    profileController->generatePath({{0_in, 0_in, 0_deg},{-25_in, 0_in, 0_deg}}, "B");
    profileController->setTarget("B", true);
    profileController->waitUntilSettled();
    forkcontroller->setTarget(1900);
    forkcontroller->waitUntilSettled();
    liftcontroller->setTarget(200);
    leftsidecontroller->setTarget(motor::front_left.get_position()-1000);
    rightsidecontroller->setTarget(motor::front_right.get_position()+1000);
    leftsidecontroller->waitUntilSettled();
    profileController->generatePath({{0_in, 0_in, 0_deg},{-10_in, 0_in, 0_deg}}, "C");
    profileController->waitUntilSettled();
    // profileController->generatePath({{0_in, 0_in, 0_deg}, {-10_in, 0_in, 0_deg}}, "D");
    // profileController->waitUntilSettled();
    // profileController->setTarget("D", true);
    pros::delay(1000);
}

void right_auton2() {
    int time = 0;
    double distance = 45; // inches to goals
    bool reached_target = false;
    
    while (time <= 15000) {
        if (!reached_target && tracking::get_distance() > distance) {
            reached_target = true;
            motor::claw.move(-75);
            motor::front_left.move(0);
            motor::back_left.move(0);
            motor::front_right.move(0);
            motor::back_right.move(0);
            pros::delay(500);
            time+=500;
        }
        if (tracking::get_distance() > distance) reached_target = true;
        if (reached_target) {
            double power = (tracking::get_distance() / distance) * 100;
            power = std::fmin(100, power);
            motor::front_left.move(-power);
            motor::back_left.move(-power);
            motor::front_right.move(-power);
            motor::back_right.move(-power);
            //motor::claw.move(-75);
            
        }
        else {
            motor::front_left.move(100);
            motor::back_left.move(100);
            motor::front_right.move(100);
            motor::back_right.move(100);
            motor::claw.move(85);
        }
        motor::lift.move(-20);
        time += 20;
        pros::delay(20);
    }
}
void left_auton1() {
    //pros::delay(2000);
    claw::hold = false;
    motor::claw.move_absolute(1000, 100);
    lift::hold = false;
    motor::lift.move(-40);
    while (tracking::get_distance() < 44) {
        drive::setDrive(127, 127);
    }
    motor::lift.move(0);
    lift::hold = true;
    drive::setDrive(0, 0);
    // drive::setDrive(1000, 1000);
    // pros::delay(1020);
    // drive::setDrive(1000, 1000);
    //drive::move_forward(47);
    motor::claw.move_absolute(325, 1000);
    pros::delay(250);
    claw::hold = true;
    drive::move_forward(-40, 0);
    claw::hold_back = false;
    motor::back_claw.move_absolute(-1500, 100);
    lift::hold = false;
    motor::lift.move_absolute(550, 100);
    pros::delay(100);
    drive::turn_radians(-pi/6);
    lift::hold = true;
    drive::move_forward(-5);
    drive::turn_radians(pi/3);
    drive::setDrive(-75, -75);
    pros::delay(600);
    drive::setDrive(0, 0);
    //drive::move_forward(-5);
    motor::back_claw.move_absolute(-200, 100);
    motor::intake.move(115);
    //claw::hold = true;
    // motor::claw.move(50);
    // pros::delay(1000);
    // motor::claw.move(-50);
    // pros::delay(1000);
}   
void right_auton1() {
    claw::hold = false;
    motor::claw.move_absolute(900, 100);
    drive::move_forward(43);
    motor::claw.move_absolute(375, 1000);
    pros::delay(350);
    claw::hold = true;
    drive::move_forward(-38);
    claw::hold_back = false;
    motor::back_claw.move_absolute(-1500, 100);
    lift::hold = false;
    motor::lift.move_absolute(500, 100);
    pros::delay(100);
    //drive::turn_radians(-pi/6);
    lift::hold = true;
    //drive::move_forward(-5);
    drive::turn_radians(pi*.35);
    drive::setDrive(-75, -75);
    pros::delay(500);
    drive::setDrive(0, 0);
    //drive::move_forward(-5);
    motor::back_claw.move_absolute(-200, 100);
    motor::intake.move(115);
}
void skills_auton() {
    //grab allaince goal
    claw::hold_back = false;
    drive::setDrive(-75, -75);
    pros::delay(250);
    drive::setDrive(0, 0);
    motor::back_claw.move_absolute(400,100);
    pros::delay(500);
    claw::hold_back = true;
    drive::move_forward(8);
    drive::turn_radians(-pi/2);
    //drive to neutral
    //drive::move_forward(48);
    claw::hold = false;
    motor::claw.move_absolute(900, 100);
    drive::move_forward(43);
    motor::claw.move_absolute(375, 1000);
    pros::delay(350);
    claw::hold = true;
    lift::hold = false;
    motor::lift.move_absolute(500, 1000);
    pros::delay(250);
    lift::hold = true;
    //drive to platform
    drive::move_forward(24);
    drive::turn_radians(pi/2);
    drive::move_forward(36);
    drive::turn_radians(-pi/2);
    //stack neutral
    lift::hold = false;
    motor::lift.move_absolute(2800, 1000);
    drive::move_forward(12);
    motor::lift.move_absolute(2400, 1000);
    lift::hold = true;
    claw::hold = false;
    motor::claw.move_absolute(900, 80);
    pros::delay(250);
    lift::hold = false;
    motor::lift.move_absolute(2800, 1000);
    claw::hold = true;
    //stack alliance
    drive::move_forward(-5);
    motor::lift.move_absolute(5,100);
    drive::turn_radians(-pi/2);
    claw::hold_back = false;
    motor::back_claw.move_absolute(0, 1000);
    drive::move_forward(3);
    drive::turn_radians(pi);
    drive::move_forward(3);
    claw::hold = false;
    motor::claw.move_absolute(375, 100);
    pros::delay(350);
    claw::hold = true;
    drive::turn_radians(pi/2);
    motor::lift.move_absolute(2800, 1000);
    pros::delay(300);
    drive::move_forward(5);
    motor::lift.move_absolute(2400, 1000);
    claw::hold = false;
    motor::claw.move_absolute(900, 100);
    pros::delay(300);
    claw::hold = true;
    // turn and stack tall on other platform
    drive::move_forward(-24);
    motor::lift.move_absolute(5, 1000);
    drive::turn_radians(pi);
    drive::move_forward(8);
    claw::hold = false;
    motor::claw.move_absolute(375, 100);
    pros::delay(300);
    claw::hold = true; 
    motor::lift.move_absolute(2800, 1000);
    pros::delay(200);
    drive::move_forward(50);
    motor::lift.move_absolute(2400, 1000);
    claw::hold = false;
    motor::claw.move_absolute(900, 100);
    pros::delay(300);
    claw::hold = true; 
    //turn and back clamp other alliance goal
    drive::move_forward(-8);
    drive::turn_radians(-pi/2);
    drive::move_forward(-30);
    claw::hold_back = false;
    motor::back_claw.move_absolute(400, 1000);
    pros::delay(500);
    //turn to stack last neutral goal
    drive::move_forward(12);
    drive::turn_radians(-pi/2);
    drive::move_forward(30);
    claw::hold = false;
    motor::claw.move_absolute(375, 100);
    pros::delay(350);
    claw::hold = true;
    lift::hold = false;
    motor::lift.move_absolute(500, 1000);
    pros::delay(250);
    lift::hold = true;
    //drive to platform
    drive::move_forward(24);
    drive::turn_radians(pi/2);
    drive::move_forward(36);
    drive::turn_radians(-pi/2);
    //stack neutral
    lift::hold = false;
    motor::lift.move_absolute(2800, 1000);
    drive::move_forward(12);
    motor::lift.move_absolute(2400, 1000);
    lift::hold = true;
    claw::hold = false;
    motor::claw.move_absolute(900, 80);
    pros::delay(250);
    lift::hold = false;
    motor::lift.move_absolute(2800, 1000);
    claw::hold = true;
}