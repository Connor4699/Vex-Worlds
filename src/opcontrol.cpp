#include "main.h"

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
		pros::lcd::set_text(1, "right");
		pros::lcd::set_text(2, std::to_string(encoder_right.get_value()));
		pros::lcd::set_text(3, "left");
		pros::lcd::set_text(4, std::to_string(encoder_left.get_value()));
		pros::lcd::set_text(5, "rear");
		pros::lcd::set_text(6, std::to_string(encoder_rear.get_value()));
		int x = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		int y = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		double armPos = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));


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