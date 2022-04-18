#include "main.h"

double circumference = pi*4.125;
double rotations = 24/circumference;
double ticks = rotations*360;

void my_autonomous() {
    // front_left.move_absolute(ticks, 50);
    // front_right.move_absolute(ticks, 50);
    // back_left.move_absolute(ticks, 50);
    // back_right.move_absolute(ticks, 50);
    // pros::delay(2000);
    // pros::lcd::set_text(1, "left (in.): " + std::to_string((encoder_left.get_value())/(360/(2.75*pi))));
	// pros::lcd::set_text(2, "right (in.): " + std::to_string((encoder_right.get_value())/(360/(2.75*pi))));
    // pros::delay(10000000);
}