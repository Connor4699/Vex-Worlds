#include "main.h"

namespace lift {
    void op_lift();
}

namespace claw {
    void op_claw();
	void op_back_claw();
}

namespace intake {
	extern bool run;
	void op_intake();
}

void motor_hold_task();