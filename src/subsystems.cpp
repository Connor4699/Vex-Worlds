#include "main.h"

namespace lift {
    void op_lift() {
        const int deadband = 5;
        double armPos = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
        int power = 0;

		if (armPos > deadband) {
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
        if (Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) {
			power = -power;
		}
        motor::lift.move(power);
    }
}

namespace claw {
    void op_claw() {
        if (Master.get_digital(DIGITAL_R1)) {
			motor::claw.move(85);
		}
		else if (Master.get_digital(DIGITAL_R2)) {
			motor::claw.move(-60);
		}
		else {
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
	
}