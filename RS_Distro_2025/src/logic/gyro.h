/***************************************************
 CSC C85 RoboSoccer gyro sensor read

***************************************************/

#ifndef LOGIC_GYRO_H
#define LOGIC_GYRO_H

// Reads the gyro's current rotation rate in degrees/second. Returns 1 and
// writes *rate_deg_s on success. returns 0 (and leaves *rate_deg_s
// and *angle_s alone) on a failed read OR a reading that looks like garbage
int gyro_read_rate(double* angle_s, double *rate_deg_s);

#endif
