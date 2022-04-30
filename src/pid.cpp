#include "main.h"

PIDController::PIDController(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    target = 0;
    output = 0;
    prev_error = 0;
    error_sum = 0;
}

void PIDController::update(double curr_val) {
    double error = target-curr_val;
    if ((error >= 0 && prev_error >= 0) || (error <= 0 && prev_error <= 0)) {
        error_sum += error;
    }
    else {
        error_sum = error;
    }
    double p = kP * error;
    double i = kI * error_sum;
    i = std::fmin(i, 30);
    i = std::fmax(i, -30);
    double d = kD * (error-prev_error);
    output = p + i + d;
    prev_error = error;
}

void PIDController::set_target(double target) {
    this->target = target;
}

bool PIDController::reached_target(double curr_val) {
    return target == curr_val || abs(target-curr_val) < 0.005;
}

void PIDController::reset() {
    target = 0;
    output = 0;
    prev_error = 0;
    error_sum = 0;
}