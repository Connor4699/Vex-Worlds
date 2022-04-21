#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(10, "I was pressed!");
	} else {
		pros::lcd::clear_line(10);
	}
}

void inertial_turn(int degrees) {
	inertial_sensor.reset();
	if (degrees > 0) {
		while (inertial_sensor.get_heading() < 90) {
			front_left.set_voltage_limit(-100);
			back_left.set_voltage_limit(-100);
			front_right.set_voltage_limit(100);
			front_right.set_voltage_limit(100);
		}
	}
	else {
        while (inertial_sensor.get_heading() < 90) {
            front_left.set_voltage_limit(100);
            back_left.set_voltage_limit(100);
            front_right.set_voltage_limit(-100);
            front_right.set_voltage_limit(-100);
        }
	}
    front_left.set_voltage_limit(0);
    back_left.set_voltage_limit(0);
    front_right.set_voltage_limit(0);
    front_right.set_voltage_limit(0);
}

// void DriveStraight(int distance) {
// 	if (distance > 0) {
// 		while (encoder_left.get_value() < distance && encoder_right.get_value() < distance) {
// 			FLeft.move(100);
// 			FRight.move(100);
// 			BLeft.move(100);
// 			BRight.move(100);
// 			if (encoder_left.get_value() >= distance) {
// 				FLeft.move(0);
// 				BLeft.move(0);
// 			}
// 			if (encoder_right.get_value() >= distance) {
// 				FRight.move(0);
// 				BRight.move(0);
// 			}
// 		}
// 	}
// 	else {
// 		while (encoder_left.get_value() > distance && encoder_right.get_value() > distance) {
// 			FLeft.move(-100);
// 			FRight.move(-100);
// 			BLeft.move(-100);
// 			BRight.move(-100);
// 		}
// 	}
// }

void sensors_reset() {
    encoder_left.reset();
    encoder_right.reset();
    encoder_back.reset();
}

double* straight() {
    curPos[0] = encoder_left.get_value()*-1;
    curPos[1] = encoder_right.get_value()*-1;
    curPos[2] = encoder_back.get_value()*-1;
    return curPos;
}

double* position() {
    curPos[0] = (encoder_left.get_value())*-1;
    curPos[1] = (encoder_right.get_value())*-1;
    curPos[2] = (encoder_back.get_value())*-1;

    double n1 = curPos[0] - prePos[0];
    double n2 = curPos[1] - prePos[1];
    double n3 = curPos[2] - prePos[2];

    double x = circ * ((n1 + n2)/2);
    double y = circ * (n3 - (B * (n2 - n1)/L));
    double theta = circ * (n2 - n1)/L;

    globalPos[0] += x * cos(globalPos[2] + theta/2) - y * sin(globalPos[2] + theta/2);
    globalPos[1] += x * sin(globalPos[2] + theta/2) + y * cos(globalPos[2] + theta/2);
    globalPos[2] += theta;

    prePos[0] = curPos[0];
    prePos[1] = curPos[1];
    prePos[2] = curPos[2];

    return globalPos;
}

void reset(){
    front_left.tare_position();
    front_right.tare_position();
    back_left.tare_position();
    back_right.tare_position();
}

void setDrive(int left, int right){
    front_left.move(left);
    front_right.move(right);
    back_left.move(left);
    back_right.move(right);
}

void PIDMove(double units){
    enable = true;
	while (enable) {
		double *pos = position();
        P = units - (pos[0]);
        I += P;
        D = P - preP;

        tP = (pos[2]*1.0);
        tI += tP;
        tD = tP - preTheta;

        PID = (P * kp) + (D * kd) + (I * ki);
        tPID = (tP * tkp) + (tD * tkd) + (tI * tki);
        // pros::lcd::set_text(0, std::to_string(pos[0]));
        // pros::lcd::set_text(1, std::to_string(pos[2]));
        // pros::lcd::set_text(3, std::to_string(PID));
        // pros::lcd::set_text(4, std::to_string(tPID));

        setDrive(PID + tPID, PID - tPID);
        preP = P;
        preTheta = tP;
        if ((units == pos[0]) || (((P < 0.03) && (P > -0.03)))) {
            enable = false;
            break;
        }
	}
}

void PIDTurn(double radians) {
	size_t count = 0;
	while (true) {
		double *pos = position();

		turnP = radians - pos[2];
		turnI += turnP;
		turnD = turnP - preTurn;
		turnPID = (turnP * turnkp) + (turnD * turnkd) + (turnI * turnki);
		setDrive(1*turnPID, -1*turnPID);
		// pros::lcd::set_text(2, std::to_string(turnPID));
		// pros::lcd::set_text(3, std::to_string(pos[2]));
		// pros::lcd::set_text(4, std::to_string(count));
		if (radians == pos[2] || (abs(radians-pos[2]) < 0.01)) {
			break;
		}
		preTurn = turnP;
		pros::delay(20);
		count++;
	}
}

void drive_to_point(double x, double y) {

}