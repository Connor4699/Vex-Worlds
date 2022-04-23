#include "main.h"

double circumference = pi*4.125;
double rotations = 24/circumference;
double ticks = rotations*360;

void my_autonomous() {

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
        if (tracking::get_distance() > distance) reached_target = true;
        if (reached_target) {
            double power = (tracking::get_distance() / distance) * 100;
            power = std::fmin(100, power);
            motor::front_left.move(-power);
            motor::back_left.move(-power);
            motor::front_right.move(-power);
            motor::back_right.move(-power);
            motor::claw.move(-75);
        }
        else {
            motor::front_left.move(100);
            motor::back_left.move(100);
            motor::front_right.move(100);
            motor::back_right.move(100);
            motor::claw.move(20);
        }
        time += 20;
        pros::delay(20);
    }
}