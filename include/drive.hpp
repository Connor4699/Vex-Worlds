#include "main.h"

void on_center_button();

void inertial_turn(int degrees);

// void DriveStraight(int distance);

void sensors_reset();

double* straight();

double* position();

void reset();

void setDrive(int left, int right);

void PIDMove(double units);

void PIDTurn(double radians);

void my_autonomous();

void my_opcontrol();
