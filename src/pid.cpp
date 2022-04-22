#include "main.h"

PIDController::PIDController(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

PIDController::~PIDController() {
    
}