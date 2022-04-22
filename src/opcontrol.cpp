#include "main.h"

void my_opcontrol() {
	while (true) {
		drive::op_drive();
		lift::op_lift();
		claw::op_claw();
		claw::op_back_claw();
		intake::op_intake();
		pros::delay(20);
	}
}