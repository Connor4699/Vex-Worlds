#include "main.h"

namespace tracking {
    struct RobotPosition {
        double x, y, angle;
    };

    struct EncoderDistances {
        double left, right, back;
    };

    extern RobotPosition robot_pos;
    extern EncoderDistances enc_dist;
    extern const double ticks_per_inch;
    extern const double dist_between_wheels;
    extern const double dist_to_rear_enc;

    double ticks_to_inches(double ticks);

    double get_delta_theta(double deltaL, double deltaR);

    void update_pos();

    void track_pos();

    void reset();
}
