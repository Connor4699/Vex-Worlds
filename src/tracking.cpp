#include "main.h"

namespace tracking {
	RobotPosition robot_pos = {0, 0, 0};
	EncoderDistances enc_dist = {0, 0, 0};
	const double ticks_per_inch = 360/(2.75*pi);
	const double dist_between_wheels = 9.70;
	const double dist_to_rear_enc = 6.45; /** @todo tune this value */

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
	 * @brief this doesn't work at the moment
	 */
	void update_pos() {
		double deltaL = ticks_to_inches(enc::left.get_value()) - enc_dist.left;
		double deltaR = ticks_to_inches(enc::right.get_value()) - enc_dist.right;
		double deltaB = ticks_to_inches(enc::back.get_value()) - enc_dist.back;
		double deltaTheta = (deltaR-deltaL) / dist_between_wheels;

		enc_dist.left = ticks_to_inches(enc::left.get_value());
		enc_dist.right = ticks_to_inches(enc::right.get_value());
		enc_dist.back = ticks_to_inches(enc::back.get_value());

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
		
		double p = half_ang + robot_pos.heading; // The global ending angle of the robot
		double cosP = std::cos(p);
		double sinP = std::sin(p);

		// update the global position
		robot_pos.y += local_y * cosP;
		robot_pos.x += local_y * sinP;

		robot_pos.y += local_x * (-sinP); // -sin(x) = sin(-x)
		robot_pos.x += local_x * cosP; // cos(x) = cos(-x)

		robot_pos.heading += deltaTheta;
        robot_pos.heading = std::fmod(robot_pos.heading, (2*pi));
	}

    /**
     * @brief update the position of the robot
     * @todo I would like for this function to get tested and determine if it's correct and/or accurate at all
     */
    void update_pos2() {
        // getting positions of encoders in inches
        double posL = ticks_to_inches(enc::left.get_value());
        double posR = ticks_to_inches(enc::right.get_value());
        double posB = ticks_to_inches(enc::back.get_value());

        // calculating the change in position of the encoders in inches
        double deltaL = posL - enc_dist.left;
		double deltaR = posR - enc_dist.right;
		double deltaB = posB - enc_dist.back;
        
        // calculating the change in angle (in radians) of the robot from the last position
        double deltaTheta = (deltaL-deltaR) / dist_between_wheels;
        
        // calculating the change in position of the tracking centre in inches
        double deltaCentre = (deltaL+deltaR) / 2.0;
        
        // calculating the perpendicular displacement using dist-to-rear as the forward offset
        double deltaPerp = deltaB - (dist_to_rear_enc*deltaTheta);

        // calculating sine and cosine values of the (old) heading
        double sinHead = std::sin(robot_pos.heading);
        double cosHead = std::cos(robot_pos.heading);

        // calculating delta x and delta y
        double deltaX = deltaCentre * cosHead - deltaPerp * sinHead;
        double deltaY = deltaCentre * sinHead + deltaPerp * cosHead;

        // updating global position
        robot_pos.x += deltaX;
        robot_pos.y += deltaY;
        robot_pos.heading += deltaTheta;
		robot_pos.heading = std::fmod(robot_pos.heading, (2*pi));

        // updating encoder positions
		enc_dist.left = posL;
		enc_dist.right = posR;
		enc_dist.back = posB;
    }

	void positionTracking() {
		// getting positions of encoders in inches
        double posL = ticks_to_inches(enc::left.get_value());
        double posR = ticks_to_inches(enc::right.get_value());
        double posB = ticks_to_inches(enc::back.get_value());

        // calculating the change in position of the encoders in inches
        double deltaL = posL - enc_dist.left;
		double deltaR = posR - enc_dist.right;
		double deltaB = posB - enc_dist.back;

		// updating encoder positions
		enc_dist.left = posL;
		enc_dist.right = posR;
		enc_dist.back = posB;

		// calculate the change in the angle of the bot (RADIANS)
		double deltaTheta = (deltaR-deltaL) / dist_between_wheels;


		double deltaXLocal, deltaYLocal;
		//If we didn't turn, then we only translated
		if(deltaTheta == 0) {
			deltaXLocal = deltaB;
		// could be either L or R, since if deltaTheta == 0 we assume they're =
			deltaYLocal = deltaL;
		}
		//Else, caluclate the new local position
		else {
		//Calculate the changes in the X and Y values (INCHES)
		//General equation is:
			//Distance = 2 * Radius * sin(deltaTheta / 2)
			deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaB / deltaTheta) + dist_between_wheels/2.0);
			deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaR / deltaTheta) - dist_to_rear_enc);

		}

		//The average angle of the robot during it's arc (RADIANS)
		double avgThetaForArc = robot_pos.heading + (deltaTheta / 2);

		double deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
		double deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

		robot_pos.heading += deltaTheta;

		//Wraps angles back around if they ever go under 0 or over 2 pi
		while(robot_pos.heading >= 2 * pi) {
			robot_pos.heading -= 2 * pi;
		}
		
		while(robot_pos.heading < 0) {
			robot_pos.heading += 2 * pi;
		}

		//Update global positions
		robot_pos.x += deltaXGlobal;
		robot_pos.y += deltaYGlobal;
	}

	/**
	 * @brief 
	 * 
	 */
	void track_pos() {
		reset();
		while (true) {
			update_pos();
            //update_pos2();
			//positionTracking();
			pros::lcd::set_text(1, "radians: " + std::to_string(robot_pos.heading));
			pros::lcd::set_text(2, "x: " + std::to_string(robot_pos.x));
			pros::lcd::set_text(3, "y: " + std::to_string(robot_pos.y));
			pros::lcd::set_text(4, "left (in.): " + std::to_string(enc_dist.left));
			pros::lcd::set_text(5, "right (in.): " + std::to_string(enc_dist.right));
			pros::lcd::set_text(6, "back (in.): " + std::to_string(enc_dist.back));
			pros::delay(10);
		}
	}

	void reset() {
		enc::left.reset();
		enc::right.reset();
		enc::back.reset();
		robot_pos = {0, 0, 0};
		enc_dist = {0, 0, 0};
	}

	double get_distance() {
		return (enc_dist.left+enc_dist.right)/2.0;
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
