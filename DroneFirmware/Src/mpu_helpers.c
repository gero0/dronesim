//
// Created by gero on 8/6/23.
//

#include <math.h>
#include "mpu_helpers.h"

void quat_to_float(const long quat[4], float quat_float[4]) {
    for (int i = 0; i < 4; i++) {
        quat_float[i] = (float) (quat[i]) / (2147483647.0f / 2.0f);
    }
}

void accel_to_gs(const short accel[3], float gs[3], float fsr) {
    for (int i = 0; i < 3; i++) {
        gs[i] = ((float) (accel[i]) / 32750.0f) * fsr;
    }
}

void quat_to_angles(const float quat_f[4], float angles[3]) {
    //PRY
    angles[0] = atan2f(2.0f * (quat_f[3] * quat_f[2] + quat_f[0] * quat_f[1]),
                       1.0f - 2.0f * (quat_f[1] * quat_f[1] + quat_f[2] * quat_f[2]));
    angles[1] = asinf(2.0f * (quat_f[2] * quat_f[0] - quat_f[3] * quat_f[1]));
    angles[2] = atan2f(2.0f * (quat_f[3] * quat_f[0] + quat_f[1] * quat_f[2]),
                       -1.0f + 2.0f * (quat_f[0] * quat_f[0] + quat_f[1] * quat_f[1]));
}

void gyro_to_dps(const short *gyro, float *dps, float fsr) {
    for (int i = 0; i < 3; i++) {
        dps[i] = ((float) (gyro[i]) / 32750.0f) * fsr;
    }
}
