//
// Created by gero on 8/6/23.
//

#ifndef MPUTEST_MPU_HELPERS_H
#define MPUTEST_MPU_HELPERS_H

#ifdef __cplusplus
extern "C"{
#endif

void quat_to_float(const long quat[4], float quat_float[4]);
void quat_to_angles(const float quat[4], float angles[3]);
void accel_to_gs(const short accel[3], float gs[3], float fsr);
void gyro_to_dps(const short gyro[3], float dps[3], float fsr);

#ifdef __cplusplus
}
#endif

#endif //MPUTEST_MPU_HELPERS_H
