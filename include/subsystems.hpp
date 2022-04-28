#include "main.h"

namespace lift {
    extern bool hold;
    void op_lift();
}

namespace claw {
    extern bool hold, hold_back;
    void op_claw();
	void op_back_claw();
}

namespace intake {
	extern bool run;
	void op_intake();
}

void lift_hold();
void claw_hold();
void back_claw_hold();