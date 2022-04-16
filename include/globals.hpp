#include "main.h"

pros::ADIEncoder encoder_right;
pros::ADIEncoder encoder_left;
pros::ADIEncoder encoder_rear;

pros::Motor front_left;
pros::Motor front_right;
pros::Motor back_left;
pros::Motor back_right;

pros::Motor intake;
pros::Motor lift;
pros::Motor claw;
pros::Motor back_claw;

pros::Imu inertial_sensor;
pros::Controller Master;

std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<AsyncMotionProfileController> profileController;
std::shared_ptr<AsyncPositionController<double, double>> rightsidecontroller;
std::shared_ptr<AsyncPositionController<double, double>> leftsidecontroller;
std::shared_ptr<AsyncPositionController<double, double>> jawcontroller;
std::shared_ptr<AsyncPositionController<double, double>> liftcontroller;
std::shared_ptr<AsyncPositionController<double, double>> forkcontroller;

const double p;
const double circ;
const double L;
const double B;

double prePos[3];
double curPos[3];

double globalPos[3];

double P, tP, turnP;
double tD, tI, D, I, turnD, turnI, preTheta, preP, preTurn, tPID, PID, turnPID;
bool enable;
const double kp;
const double kd;
const double ki;

const double tkp;
const double tkd;
const double tki;

const double turnkp;
const double turnkd;
const double turnki;