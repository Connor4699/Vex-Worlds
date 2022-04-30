#include "main.h"

namespace drive {
    PIDController turn_PID = PIDController(70, 0.25, 0);
    PIDController drive_PID = PIDController(0, 0, 0);

    void op_drive() {
        const int deadband = 5;
        int x = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
		int y = abs(Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		double LjoyY = y/10.0;
		double LjoyX = x/10.0;
        double power = 0;
        double turn = 0;

		if (y > deadband || x > deadband) {
			if (y > 85) {
				power = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 3.0;
			}
			else if (y > 55 && y <= 85) {
				power = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyY - 6.0)))) * 10.0 - 10.0;
			}
			else {
				power = 5*pow((1.0/5.5)*(LjoyY), 3.0) * 12.7;
			}

			if (x > 85) {
				turn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 3.0;
			}
			else if (x > 55 && x <= 85) {
				turn = (12.7 / (1.0 + exp(-(3.0/4.0)*(LjoyX - 6.0)))) * 10.0 - 10.0;
			}
			else {
				turn = 5*pow((1.0/5.5)*(LjoyX), 3.0) * 12.7;
			}
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) {
			power = -power;
		}

		if (Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0) {
			turn = -turn;
		}

        double leftPower = power - 0.6*turn;
		double rightPower = power + 0.6*turn;

		motor::front_left.move(rightPower);
		motor::back_left.move(rightPower);
		motor::front_right.move(leftPower);
		motor::back_right.move(leftPower);
    }
    
    void inertial_turn(int degrees) {
        inertial_sensor.reset();
        if (degrees > 0) {
            while (inertial_sensor.get_heading() < 90) {
                motor::front_left.set_voltage_limit(-100);
                motor::back_left.set_voltage_limit(-100);
                motor::front_right.set_voltage_limit(100);
                motor::front_right.set_voltage_limit(100);
            }
        }
        else {
            while (inertial_sensor.get_heading() < 90) {
                motor::front_left.set_voltage_limit(100);
                motor::back_left.set_voltage_limit(100);
                motor::front_right.set_voltage_limit(-100);
                motor::front_right.set_voltage_limit(-100);
            }
        }
        motor::front_left.set_voltage_limit(0);
        motor::back_left.set_voltage_limit(0);
        motor::front_right.set_voltage_limit(0);
        motor::front_right.set_voltage_limit(0);
    }

    void sensors_reset() {
        enc::left.reset();
        enc::right.reset();
        enc::back.reset();
    }

    double* straight() {
        curPos[0] = enc::left.get_value()*-1;
        curPos[1] = enc::right.get_value()*-1;
        curPos[2] = enc::back.get_value()*-1;
        return curPos;
    }

    double* position() {
        curPos[0] = (enc::left.get_value())*-1;
        curPos[1] = (enc::right.get_value())*-1;
        curPos[2] = (enc::back.get_value())*-1;

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
        motor::front_left.tare_position();
        motor::front_right.tare_position();
        motor::back_left.tare_position();
        motor::back_right.tare_position();
    }

    void setDrive(int left, int right){
        motor::front_left.move(left);
        motor::front_right.move(right);
        motor::back_left.move(left);
        motor::back_right.move(right);
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
        double changeX = tracking::get_x() - x;
        double changeY = tracking::get_y() - y;
        
    }

    void turn_radians(double angle) {
        turn_PID.reset();
        turn_PID.set_target(tracking::robot_pos.heading + angle);
        while (true) {
            turn_PID.update(tracking::robot_pos.heading);
            setDrive(-turn_PID.output, turn_PID.output);
            if (turn_PID.reached_target(tracking::robot_pos.heading)) {
                //break;
            }
            pros::delay(10);
        }
        // while (true) {
        //     turnP = angle - tracking::robot_pos.heading;
        //     turnI += turnP;
        //     turnD = turnP - preTurn;
        //     turnPID = (turnP * turnkp) + (turnD * turnkd) + (turnI * turnki);
        //     setDrive(-1*turnPID, 1*turnPID);
        //     if (angle == tracking::robot_pos.heading || (abs(angle-tracking::robot_pos.heading) < 0.01)) {
        //         break;
        //     }
        //     preTurn = turnP;
        //     pros::delay(20);
        // }
    }
        

    void move(double inches) {

    }
}

