#include "main.h"

namespace drive {
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
    void my_autonomous();
    void my_opcontrol();
}


