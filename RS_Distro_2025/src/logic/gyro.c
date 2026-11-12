#include "gyro.h"
#include "../roboAI.h" // GYRO_PORT

// Implausible for this robot even in a violent spin/collision.
// Guards against the documented garbage-read pattern on this sensor/library.
#define GYRO_RATE_SANITY_LIMIT_DEGS 2000.0

int gyro_read_rate(double* angle_s, double *rate_deg_s) {
    int angle = 0, rate = 0;
    if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        return 0;
    if (rate > GYRO_RATE_SANITY_LIMIT_DEGS || rate < -GYRO_RATE_SANITY_LIMIT_DEGS)
        return 0;
    *angle_s = (double) angle;
    *rate_deg_s = (double)rate;
    return 1;
}
