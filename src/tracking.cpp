#include "main.h"

namespace tracking {
	RobotPosition robot_pos = {0, 0, 0}; // x (inches), y (inches), heading (radians)
	EncoderDistances enc_pos = {0, 0, 0}; // left, right, back (all in encoder ticks)
	const double inches_per_tick = (2.75*pi)/360;
	const double dist_between_wheels = 9.70;
	const double dist_to_rear_enc = 6.45; /** @todo tune this value */

	void update_pos() {
		// getting current encoder positions
		double posL = enc::left.get_value();
		double posR = enc::right.get_value();
		double posB = enc::back.get_value();

		// getting change in encoder positions
		double deltaL = posL - enc_pos.left;
		double deltaR = posR - enc_pos.right;
		double deltaB = posB - enc_pos.back;

		// calculating values
		double theta = inches_per_tick * (deltaR-deltaL) / dist_between_wheels;
		double localX = inches_per_tick * (deltaR+deltaL) / 2.0;
		double localY = inches_per_tick * (deltaB - dist_to_rear_enc * theta);
		double avgAngle = robot_pos.heading + theta / 2.0;

		// updating global position
		robot_pos.x += localX * cos(avgAngle) - localY * sin(avgAngle);
		robot_pos.y += localX * sin(avgAngle) + localY * cos(avgAngle);
		robot_pos.heading += theta;

		// updating encoder positions
		enc_pos.left = posL;
		enc_pos.right = posR;
		enc_pos.back = posB;
	}

	void track_pos() {
		reset();
		while (true) {
			update_pos();
			pros::lcd::set_text(1, "x (in.): " + std::to_string(robot_pos.x));
			pros::lcd::set_text(2, "y (in.): " + std::to_string(robot_pos.y));
			pros::lcd::set_text(3, "heading (radians): " + std::to_string(robot_pos.heading));
			pros::lcd::set_text(4, "left (ticks): " + std::to_string(enc_pos.left));
			pros::lcd::set_text(5, "right (ticks): " + std::to_string(enc_pos.right));
			pros::lcd::set_text(6, "back (ticks): " + std::to_string(enc_pos.back));
			pros::delay(10);
		}
	}

	void reset() {
		enc::left.reset();
		enc::right.reset();
		enc::back.reset();
		robot_pos = {0, 0, 0};
		enc_pos = {0, 0, 0};
	}

	double get_distance() {
		return (enc_pos.left+enc_pos.right)/2.0;
	}

	double get_x() {
		return robot_pos.x;
	}

	double get_y() {
		return robot_pos.y;
	}

	double get_heading() {
		return robot_pos.heading;
	}
}
