#include "main.h"

namespace lift {
	bool hold = true;
    void op_lift() {
        const int deadband = 5;
        double armPos = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        int power = 0;

		if (armPos > deadband) {
			hold = false;
			if (armPos > 85) {
				power = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 3.0) * 1;
			}
			else if (armPos > 55 && armPos <= 85 ) {
				power = ((12.7 / (1.0 + exp(-(3.0/4.0)*(armPos/10.0 - 6.0)))) * 10.0 - 10.0) * 1;
			}
			else {
				power = (5*pow((1.0/5.5)*(armPos/10.0), 3.0) * 12.7) * 1;
			}
		}
		else {
			hold = true;
			return;
		}
        if (Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) {
			power = -power;
		}
        motor::lift.move(power);
    }
}

namespace claw {
	bool hold = true, hold_back = true;
    void op_claw() {
        if (Master.get_digital(DIGITAL_R1)) {
			hold = false;
			motor::claw.move(85);
		}
		else if (Master.get_digital(DIGITAL_R2)) {
			hold = false;
			motor::claw.move(-60);
		}
		else {
			hold = true;
			motor::claw.move(0);
		}
    }

	void op_back_claw() {
		if (Master.get_digital(DIGITAL_L2)) {
			hold_back = false;
			motor::back_claw.move(127);
		}
		else if (Master.get_digital(DIGITAL_L1)){
			hold_back = false;
			motor::back_claw.move(-127);
		}
		else {
			hold_back = true;
			motor::back_claw.move(0);
		}
	}
}

namespace intake {
	bool run = true;
	void op_intake() {
		if (Master.get_digital(DIGITAL_A)) {
			if (run) {
				run = false;
			}
			else {
				run = true;
			}
		}
		if (motor::lift.get_position() >= 700 && run) {
			motor::intake.move(115);
		}
		else {
			motor::intake.move(0);
		}
	}
}

void lift_hold() {
	while (true) {
		if (lift::hold) {
			int absPos = motor::lift.get_position();
			while (lift::hold) {
				motor::lift = absPos - motor::lift.get_position();
				pros::delay(20);
			}
		}
		pros::delay(20);
	}
}

void claw_hold() {
	while (true) {
		if (claw::hold) {
			int absPos = motor::claw.get_position();
			while (claw::hold) {
				motor::claw = absPos - motor::claw.get_position();
				pros::delay(20);
			}
		}
		pros::delay(20);
	}
}

void back_claw_hold() {
	while (true) {
		if (claw::hold_back) {
			int absPos = motor::back_claw.get_position();
			while (claw::hold_back) {
				motor::back_claw = absPos - motor::back_claw.get_position();
				pros::delay(20);
			}
		}
		pros::delay(20);
	}
}