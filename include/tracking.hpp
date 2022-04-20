#include "main.h"

struct RobotPosition {
    double x, y, angle;
};

struct EncoderDistances {
    double left, right, back;
};

RobotPosition robot_pos = {0, 0, 0};
EncoderDistances enc_dist = {0, 0, 0};
const double ticks_per_inch = 360/(2.75*pi);
const double dist_between_wheels = 9.25;
const double dist_to_rear_enc = 3;

double ticks_to_inches(double ticks);

double get_delta_theta(double deltaL, double deltaR);

void update_pos();

void track_pos();

void reset();