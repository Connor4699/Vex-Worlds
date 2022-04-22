#include "main.h"

namespace tracking {
	RobotPosition robot_pos = {0, 0, 0};
	EncoderDistances enc_dist = {0, 0, 0};
	const double ticks_per_inch = 360/(2.75*pi);
	const double dist_between_wheels = 9.24;
	const double dist_to_rear_enc = 2.5; /** @todo tune this value */

	/**
	 * @brief 
	 * 
	 * @param ticks encoder ticks
	 * @return number of inches traveled
	 */
	double ticks_to_inches(double ticks) {
		return ticks/ticks_per_inch;
	}

	/**
	 * @brief Get the change in angle (in radians) of the robot from the last position
	 * 
	 * @return the change in angle (in radians) of the robot from the last position
	 */
	double get_delta_theta(double deltaL, double deltaR) {
		return (deltaL-deltaR)/dist_between_wheels;
	}

	/**
	 * @brief 
	 * 
	 */
	void update_pos() {
		double deltaL = ticks_to_inches(encoder_left.get_value()) - enc_dist.left;
		double deltaR = ticks_to_inches(encoder_right.get_value()) - enc_dist.right;
		double deltaB = ticks_to_inches(encoder_back.get_value()) - enc_dist.back;
		double deltaTheta = get_delta_theta(deltaL, deltaR);
		
		//robot_position[2] = std::fmod(robot_position[2], (2*pi));

		enc_dist.left = ticks_to_inches(encoder_left.get_value());
		enc_dist.right = ticks_to_inches(encoder_right.get_value());
		enc_dist.back = ticks_to_inches(encoder_back.get_value());

		double local_x, local_y, half_ang;

		if (deltaTheta) {
			half_ang = deltaTheta/2.0;
			double r = deltaB/deltaTheta;
			double r2 = deltaR/deltaTheta;
			local_x = 2.0 * std::sin(half_ang) * (r + dist_to_rear_enc); // h2
			local_y = 2.0 * std::sin(half_ang) * (r2 + dist_between_wheels/2.0); // h
		}
		else {
			local_y = deltaR;
			local_x = deltaB;
			half_ang = 0;
		}
		
		double p = half_ang + robot_pos.angle; // The global ending angle of the robot
		double cosP = std::cos(p);
		double sinP = std::sin(p);

		// update the global position
		robot_pos.y += local_y * cosP;
		robot_pos.x += local_y * sinP;

		robot_pos.y += local_x * (-sinP); // -sin(x) = sin(-x)
		robot_pos.x += local_x * cosP; // cos(x) = cos(-x)

		robot_pos.angle += deltaTheta;
	}

	/**
	 * @brief 
	 * 
	 */
	void track_pos() {
		while (true) {
			update_pos();
			pros::lcd::set_text(1, "radians: " + std::to_string(robot_pos.angle));
			pros::lcd::set_text(2, "x: " + std::to_string(robot_pos.x));
			pros::lcd::set_text(3, "y: " + std::to_string(robot_pos.y));
			pros::lcd::set_text(4, "left (in.): " + std::to_string(enc_dist.left));
			pros::lcd::set_text(5, "right (in.): " + std::to_string(enc_dist.right));
			pros::lcd::set_text(6, "back (in.): " + std::to_string(enc_dist.back));
			pros::delay(20);
		}
	}

	void reset() {
		
	}
}
