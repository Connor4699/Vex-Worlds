#include "main.h"

struct PIDController {
    double kP, kI, kD;
    PIDController(double kP, double kI, double kD);
    virtual ~PIDController();
};