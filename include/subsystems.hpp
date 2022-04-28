#include "main.h"

namespace lift {
    extern bool hold;
    void op_lift();
}

namespace claw {
    extern bool hold;
    void op_claw();
	void op_back_claw();
}

namespace intake {
	extern bool run;
	void op_intake();
}

void motor_hold_task();
void claw_hold_task();
void back_claw_hold_task();