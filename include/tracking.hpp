#include "main.h"

namespace tracking {
    struct RobotPosition {
        double x, y, heading;
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

    void update_pos();

    void update_pos2();

    void track_pos();

    void reset();

    double get_distance();
}
