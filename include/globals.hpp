#include "main.h"

extern pros::ADIEncoder encoder_right;
extern pros::ADIEncoder encoder_left;
extern pros::ADIEncoder encoder_back;

extern pros::Motor front_left;
extern pros::Motor front_right;
extern pros::Motor back_left;
extern pros::Motor back_right;

extern pros::Motor intake;
extern pros::Motor lift;
extern pros::Motor claw;
extern pros::Motor back_claw;

extern pros::Imu inertial_sensor;
extern pros::Controller Master;

extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<AsyncMotionProfileController> profileController;
extern std::shared_ptr<AsyncPositionController<double, double>> rightsidecontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> leftsidecontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> jawcontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> liftcontroller;
extern std::shared_ptr<AsyncPositionController<double, double>> forkcontroller;

extern const double p;
extern const double circ;
extern const double L;
extern const double B;

extern double prePos[3];
extern double curPos[3];

extern double globalPos[3];

extern double P, tP, turnP;
extern double tD, tI, D, I, turnD, turnI, preTheta, preP, preTurn, tPID, PID, turnPID;
extern bool enable;
extern const double kp;
extern const double kd;
extern const double ki;

extern const double tkp;
extern const double tkd;
extern const double tki;

extern const double turnkp;
extern const double turnkd;
extern const double turnki;
