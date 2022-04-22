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
	bool hold = true;
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
			motor::back_claw.move_velocity(100);
		}
		else if (Master.get_digital(DIGITAL_L1)){
			motor::back_claw.move_velocity(-100);
		}
		else {
			motor::back_claw.move_velocity(0);
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
		if (motor::lift.get_position() >= 400 && run) {
			motor::intake.move(100);
		}
		else {
			motor::intake.move(0);
		}
	}
}

void motor_hold_task() {
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