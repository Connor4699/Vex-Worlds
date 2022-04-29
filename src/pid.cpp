#include "main.h"

PIDController::PIDController(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    target = 0;
    output = 0;
    prev_error = 0;
}

void PIDController::update(double curr_val) {
    double error = target-curr_val;
    double p = kP * error;
    double d = kD * (error-prev_error);
    output = p + d;
    prev_error = error;
}

void PIDController::set_target(double target) {
    this->target = target;
}

bool PIDController::reached_target(double curr_val) {
    return target == curr_val || abs(target-curr_val) < 0.01;
}

void PIDController::reset() {
    target = 0;
    output = 0;
    prev_error = 0;
}