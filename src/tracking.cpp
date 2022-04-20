#include "main.h"

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
 * @brief Get the delta theta 
 * 
 * @return the change in theta from the last position
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

	double local_x = 2 * std::sin(robot_pos.angle/2) * (deltaB/deltaTheta + dist_to_rear_enc);
	double local_y = 2 * std::sin(robot_pos.angle/2) * (deltaR/deltaTheta + dist_between_wheels/2);
	
	double p = deltaTheta/2 + robot_pos.angle; // The global ending angle of the robot
	double cosP = std::cos(p);
	double sinP = std::sin(p);

	// Update the global position
	robot_pos.y += local_y * cosP;
	robot_pos.x += local_y * sinP;

	robot_pos.y += local_x * -sinP; // -sin(x) = sin(-x)
	robot_pos.x += local_x * cosP; // cos(x) = cos(-x)

	// update global positions
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
		pros::lcd::set_text(2, "left (in.): " + std::to_string(enc_dist.left));
		pros::lcd::set_text(3, "right (in.): " + std::to_string(enc_dist.right));
		pros::lcd::set_text(4, "back (in.): " + std::to_string(enc_dist.back));
        pros::delay(20);
    }
}

void reset() {
    
}