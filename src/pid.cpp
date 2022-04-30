#include "main.h"

PIDController::PIDController(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    target = 0;
    output = 0;
    prev_error = 0;
    error_sum = 0;
    limit = 0;
    acceptable_error = 0;
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
    i = std::fmin(i, limit);
    i = std::fmax(i, -limit);
    double d = kD * (error-prev_error);
    output = p + i + d;
    prev_error = error;
}

void PIDController::set_target(double target) {
    this->target = target;
}

bool PIDController::reached_target(double curr_val) {
    return target == curr_val || abs(target-curr_val) < acceptable_error;
}

void PIDController::set_limit(double limit) {
    this->limit = limit;
}

void PIDController::set_acceptable_error(double ae) {
    acceptable_error = ae;
}

void PIDController::reset() {
    target = 0;
    output = 0;
    prev_error = 0;
    error_sum = 0;
}