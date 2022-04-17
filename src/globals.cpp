#include "main.h"

pros::ADIEncoder encoder_right(7, 8, false);
pros::ADIEncoder encoder_left(5, 6, true);
pros::ADIEncoder encoder_rear(3, 4, true);

pros::Motor front_left(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor front_right(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor back_left(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor back_right(13, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor intake(14, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor lift(18, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor claw(1, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor back_claw(15, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Imu inertial_sensor(14);
pros::Controller Master (pros::E_CONTROLLER_MASTER);

std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors({19, 20}, {11, 13}) // left motor is 1, right motor is 2 (reversed)
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
   		// .withGains({liftkP, liftkI, liftkD})
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

const double p = 2 * acos(0.0);
const double circ = (p * 2.75)/360;
const double L = 9.25;
const double B = 3;

//left, right, back, heading encoder values
double prePos[3] = {0, 0, 0};
double curPos[3] = {0, 0, 0};

//x position, y position, heading
double globalPos[3] = {0, 0, 0};

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
