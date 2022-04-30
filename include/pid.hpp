#pragma once
#include "main.h"

class PIDController {
public:
    double kP;
    double kI;
    double kD;
    double target;
    double output;
    double prev_error;
    double error_sum;
    double limit;
    double acceptable_error;
    
    PIDController(double kP, double kI, double kD);
    void update(double curr_val);
    void set_target(double target);
    bool reached_target(double curr_val);
    void set_limit(double limit);
    void set_acceptable_error(double ae);
    void reset();
};