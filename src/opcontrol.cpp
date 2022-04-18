#include "main.h"

double prev_pos[3] = {0, 0, 0};
const double ticks_per_inch = 360/(2.75*pi);
const double dist_between_wheels = 8.75;
double angle = 0;

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

void update() {
	double deltaL = ticks_to_inches(encoder_left.get_value()) - prev_pos[0];
	double deltaR = ticks_to_inches(encoder_right.get_value()) - prev_pos[1];
	angle += get_delta_theta(deltaL, deltaR);
	prev_pos[0] = ticks_to_inches(encoder_left.get_value());
	prev_pos[1] = ticks_to_inches(encoder_right.get_value());
}

void my_opcontrol() {
    front_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	front_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	back_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	back_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	back_claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	double armPwr = 0;
	double ClawOpen = 0;
	double ClawClose = 0;
	double ForkOpen = 0;
	double ForkClose = 0;
	bool Intake01 = true;

	double Pwr = 0;
	double Trn = 0;
	double leftPower = 0;
	double rightPower = 0;
	int deadband = 5;

	//Fork.set_zero_position(0);
	while (true) {
		// pros::lcd::set_text(1, "right");
		// pros::lcd::set_text(2, std::to_string(encoder_right.get_value()));
		// pros::lcd::set_text(3, "left");
		// pros::lcd::set_text(4, std::to_string(encoder_left.get_value()));
		// pros::lcd::set_text(5, "rear");
		// pros::lcd::set_text(6, std::to_string(encoder_rear.get_value()));

		update();
		pros::lcd::set_text(1, "radians: " + std::to_string(angle));
		pros::lcd::set_text(2, "left (in.): " + std::to_string(prev_pos[0]));
		pros::lcd::set_text(3, "right (in.): " + std::to_string(prev_pos[1]));

		prev_pos[2] = ticks_to_inches(encoder_rear.get_value());
		pros::lcd::set_text(4, "back (in.): " + std::to_string(prev_pos[2]));

		int x = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		int y = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		double armPos = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		// if (Master.get_digital(DIGITAL_B)) {
		// 	front_left.move(0);
		// front_right.move(0);
		// back_left.move(0);
		// back_right.move(0);
		// 	pros::delay(10000000);
		// }
		// front_left.move(50);
		// front_right.move(50);
		// back_left.move(50);
		// back_right.move(50);
		
		double LjoyY = y/10.0;
		double LjoyX = x/10.0;
		double finalPWr = 0;

		if (y > deadband || x > deadband) {
			if (y > 85) {
				Pwr = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 3.0;
			}
			else if (y > 55 && y <= 85) {
				Pwr = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 10.0;
			}
			else {
				Pwr = 5*pow((1.0/5.5)*(LjoyY), 3.0) * 12.7;
			}

			if (x > 85) {
				Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 3.0;
			}
			else if (x > 55 && x <= 85) {
				Trn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 10.0;
			}
			else {
				Trn = 5*pow((1.0/5.5)*(LjoyX), 3.0) * 12.7;
			}
		}
		else {
			Pwr = 0;
			Trn = 0;
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) {
			Pwr = -Pwr;
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0) {
			Trn = -Trn;
		}

		if (armPos > deadband) {
			if (armPos > 85) {
				armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 3.0) * 1;
			}
			else if (armPos > 55 && armPos <= 85 ) {
				armPwr = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 10.0) * 1;
			}
			else {
				armPwr = (5*pow((1.0/5.5)*(armPos/10.0), 3.0) * 12.7) * 1;
			}
		}
		else {
			armPwr = 0;
		}

		if (Master.get_digital(DIGITAL_R1)) {
			claw.move(85);
		}
		else if (Master.get_digital(DIGITAL_R2)) {
			claw.move(-60);
		}
		else {
			claw.move(0);
		}

		if (Master.get_digital(DIGITAL_L2)) {
			//Fork.move_absolute(double (1850), 127);
			//forkcontroller->setTarget(0);
			back_claw.move_velocity(100);
		}
		else if (Master.get_digital(DIGITAL_L1)){
			//Fork.move_absolute(double (1000), 127);
			//forkcontroller->setTarget(-1900);
			back_claw.move_velocity(-100);
		}
		else {
			back_claw.move_velocity(0);
		}
		//else if (Master.get_digital(DIGITAL_UP)){
		// 	//Fork.move_absolute(double (10), 127);
		// 	forkcontroller->setTarget(15);
		//
		// }

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) {
			armPwr = -armPwr;
		}

		if (Master.get_digital(DIGITAL_A)) {
			if (Intake01) {
				Intake01 = false;
			}
			else {
				Intake01 = true;
			}
		}

		if (lift.get_position() >= 400 && Intake01) {
			intake.move(100);
		}
		else {
			intake.move(0);
		}

		leftPower = Pwr - 0.6*Trn;
		rightPower = Pwr + 0.6*Trn;

		front_left.move(rightPower);
		back_left.move(rightPower);
		front_right.move(leftPower);
		back_right.move(leftPower);
		lift.move(armPwr);
		
		pros::delay(20);
	}
}