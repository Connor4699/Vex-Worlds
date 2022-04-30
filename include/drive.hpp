#include "main.h"
#include "pid.hpp"

namespace drive {
    extern PIDController turn_PID;
    extern PIDController drive_PID;

    void op_drive();
    void on_center_button();
    void inertial_turn(int degrees);
    void sensors_reset();
    double* straight();
    double* position();
    void reset();
    void setDrive(int left, int right);
    void PIDMove(double units);
    void PIDTurn(double radians);
    void turn_radians(double angle);
    void move_forward(double inches);
}

