#include "main.h"

// pros::Motor FLeft(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor FRight(10, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor BLeft(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor BRight(14, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIEncoder encoder_right(7, 8, false);
pros::ADIEncoder encoder_left(5, 6, true);
pros::ADIEncoder encoder_rear(3, 4, true);
pros::Motor FLeft(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FRight(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BLeft(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BRight(13, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
// comment

pros::Motor Intake(14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Lift(18, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Claw(1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BClaw(15, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Imu inertial_sensor(14);
pros::Controller Master (pros::E_CONTROLLER_MASTER);

std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
	.withMotors({3, 12}, {4, 8}) // left motor is 1, right motor is 2 (reversed)
	// green gearset, 4 inch wheel diameter, 11.5 inch wheel track
	.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
	.withMaxVelocity(50)
	// left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
	.withSensors(ADIEncoder{'H', 'G'}, ADIEncoder{'D', 'C'})
	// specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
	.withOdometry({{2.75_in, 7_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
	.buildOdometry();


std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder()
	.withLimits({
		0.6, // Maximum linear velocity of the Chassis in m/s
		1.0, // Maximum linear acceleration of the Chassis in m/s/s
		2.5 // Maximum linear jerk of the Chassis in m/s/s/s
	})
	.withOutput(chassis)
	.buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> rightsidecontroller =
	AsyncPosControllerBuilder()
	.withMotor({14, 19}) // lift motor port 3
//    .withGains({liftkP, liftkI, liftkD})
	.build();

std::shared_ptr<AsyncPositionController<double, double>> leftsidecontroller =
	AsyncPosControllerBuilder()
	// .withLimits({
	// 	0.25,
	// 	0.5,
	// 	2.5
										//})
	.withMotor({20, 16}) // lift motor port 3
					//        .withGains({liftkP, liftkI, liftkD})
	.build();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
	AsyncPosControllerBuilder()
	.withMotor(15) // lift motor port 3
	//        .withGains({liftkP, liftkI, liftkD})
	.build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
	AsyncPosControllerBuilder()
	.withMotor(18) // lift motor port 3
//    .withGains({liftkP, liftkI, liftkD})
	.build();

std::shared_ptr<AsyncPositionController<double, double>> forkcontroller =
	AsyncPosControllerBuilder()
	.withMotor(1) // lift motor port 3
//    .withGains({liftkP, liftkI, liftkD})
	.build();

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
			FLeft.set_voltage_limit(-100);
			BLeft.set_voltage_limit(-100);
			FRight.set_voltage_limit(100);
			FRight.set_voltage_limit(100);
		}
	}
	else {
	while (inertial_sensor.get_heading() < 90) {
		FLeft.set_voltage_limit(100);
		BLeft.set_voltage_limit(100);
		FRight.set_voltage_limit(-100);
		FRight.set_voltage_limit(-100);
	}
	}
		FLeft.set_voltage_limit(0);
		BLeft.set_voltage_limit(0);
		FRight.set_voltage_limit(0);
		FRight.set_voltage_limit(0);
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

const double p = 2 * acos(0.0);
const double circ = (p * 2.75)/360;
const double L = 9.25;
const double B = 3;

//left, right, back, heading encoder values
double prePos[3] = {0, 0, 0};
double curPos[3] = {0, 0, 0};

//x position, y position, heading
double globalPos[3] = {0, 0, 0};

void Sensors_reset(){
		encoder_left.reset();
		encoder_right.reset();
		encoder_rear.reset();
}

double * straight(){
		curPos[0] = encoder_left.get_value()*-1;
		curPos[1] = encoder_right.get_value()*-1;
		curPos[2] = encoder_rear.get_value()*-1;
		return curPos;
}


double * position(){
		curPos[0] = (encoder_left.get_value())*-1;
		curPos[1] = (encoder_right.get_value())*-1;
		curPos[2] = (encoder_rear.get_value())*-1;

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
	 FLeft.tare_position();
	 FRight.tare_position();
	 BLeft.tare_position();
	 BRight.tare_position();
}
void setDrive(int left, int right){
	 FLeft.move(left);
	 FRight.move(right);
	 BLeft.move(left);
	 BRight.move(right);
}

double P, tP, turnP;
double tD, tI, D, I, turnD, turnI, preTheta, preP, preTurn, tPID, PID, turnPID = 0;
bool enable = true;
const double kp = 15;
const double kd = 10;
const double ki = 0;

const double tkp = 17;
const double tkd = 5;
const double tki = 0;

const double turnkp = 130;
const double turnkd = 11;
const double turnki = 0;

void PIDMove(double units){
 enable = true;
	 while(enable) {
			 double *pos = position();
			 P = units - (pos[0]);
			 I += P;
			 D = P - preP;

			 tP = (pos[2]*1.0);
			 tI += tP;
			 tD = tP - preTheta;

			 PID = (P * kp) + (D * kd) + (I * ki);
			 tPID = (tP * tkp) + (tD * tkd) + (tI * tki);
			 pros::lcd::set_text(0, std::to_string(pos[0]));
			 pros::lcd::set_text(1, std::to_string(pos[2]));
			 pros::lcd::set_text(3, std::to_string(PID));
			 pros::lcd::set_text(4, std::to_string(tPID));

			 setDrive(PID + tPID, PID - tPID);
			 preP = P;
			 preTheta = tP;
			 if ((units == pos[0]) || (((P < 0.03) && (P > -0.03)))) {
					 enable = false;
					 break;
			 }
	 }
}
void PIDTurn(double radians){
 enable = true;
	 while(enable){
			 double *pos = position();

			 turnP = radians - pos[2];
			 turnI += turnP;
			 turnD = turnP - preTurn;
			 turnPID = (turnP * turnkp) + (turnD * turnkd) + (turnI * turnki);
			 setDrive(1*turnPID, -1*turnPID);
			 pros::lcd::set_text(2, std::to_string(turnPID));
			 pros::lcd::set_text(3, std::to_string(pos[2]));
			 if (radians == pos[2] || (abs(radians-pos[2]) <= 0.01)) {
					 enable = false;
					 break;
			 }
			 preTurn = turnP;
			 pros::delay(20);
	 }
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(9, "I'm hungry!");

	pros::lcd::register_btn1_cb(on_center_button);

	//autonomous();

	// pros::Motor FLeft(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor FRight(10, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor BLeft(16, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor BRight(14, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
	//
	// pros::Motor Lift(18, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor Claw(15, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
	// pros::Motor Fork(17, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
	//
	// FLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// FRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// BLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// BRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Fork.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

//DriveStraight(1080);

//inertial_turn(90);
// set the state to zero
// chassis->setState({0_in, 0_in, 0_deg});
// // // turn 45 degrees and drive approximately 1.4 ft
// chassis->driveToPoint({4_ft, 0_ft});
// while (true) {
// 		pros::lcd::set_text(0, std::to_string(encoder_right.get_value()));
// }
// turn approximately 45 degrees to end up at 90 degrees
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	FLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	BClaw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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
			Claw.move_velocity(75);
		}
		else if (Master.get_digital(DIGITAL_R2)) {
			Claw.move_velocity(-75);
		}
		else {
			Claw.move_velocity(0);
		}

		if (Master.get_digital(DIGITAL_L2)) {
			//Fork.move_absolute(double (1850), 127);
			//forkcontroller->setTarget(0);
			BClaw.move_velocity(100);
		}
		else if (Master.get_digital(DIGITAL_L1)){
			//Fork.move_absolute(double (1000), 127);
			//forkcontroller->setTarget(-1900);
			BClaw.move_velocity(-100);
		}
		else {
			BClaw.move_velocity(0);
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

		if (Lift.get_position() >= 400 && Intake01) {
			Intake.move(100);
		}
		else {
			Intake.move(0);
		}

		leftPower = Pwr - 0.6*Trn;
		rightPower = Pwr + 0.6*Trn;

		FLeft.move(rightPower);
		BLeft.move(rightPower);
		FRight.move(leftPower);
		BRight.move(leftPower);
		Lift.move(armPwr);
		pros::delay(20);
	}
}
