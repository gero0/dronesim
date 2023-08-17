//
// Created by gero on 8/9/23.
//

#include <cmath>
#include "MPU6050Driver.h"

#define GYRO_FSR 2000
#define ACC_FSR 8

extern "C" {
#include "inv_mpu.h"
}

#include "mpu_interface.h"
#include "mpu_helpers.h"
#include "MadgwickAHRS.h"

bool MPU6050Driver::init_hardware(I2C_HandleTypeDef *i2c) {
    mpu_interface_register(i2c);
    struct int_param_s int_param;
    int result = mpu_init(&int_param);
    if (result) {
        return false;
    }
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_accel_fsr(ACC_FSR);
    mpu_set_gyro_fsr(GYRO_FSR);
    mpu_set_sample_rate(1000);

    initialized = true;

    return true;
}


bool MPU6050Driver::calibrate() const {
    if (!initialized) {
        return false;
    }
    long accel_bias[3];
    long gyro_bias[3];
    int result = mpu_run_self_test(gyro_bias, accel_bias);

    if (result != 0x7) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = (long) ((float) gyro_bias[i] * 32.8f); //convert to +-1000dps
        accel_bias[i] *= 2048.f; //convert to +-16G
        accel_bias[i] = accel_bias[i] >> 16;
        gyro_bias[i] = (long) (gyro_bias[i] >> 16);
    }

    mpu_set_gyro_bias_reg(gyro_bias);
    mpu_set_accel_bias_6050_reg(accel_bias);

    return true;
}

void MPU6050Driver::sensor_update() {
    if (!initialized) {
        return;
    }
    short accel_raw[3];
    short gyro_raw[3];
    float accel_gs[3];
    float angles[3];

    mpu_get_accel_reg(accel_raw, nullptr);
    accel_to_gs(accel_raw, (float *) accel_gs, ACC_FSR);

    acc_samples[acc_i] = {accel_gs[1], accel_gs[0], -accel_gs[2]};
    acc_i = (acc_i + 1) % averaging_len;
    acceleration_current = {0,0,0};
    for(auto acc_sample : acc_samples){
        acceleration_current += acc_sample;
    }
    acceleration_current /= averaging_len;

    mpu_get_gyro_reg(gyro_raw, nullptr);
    gyro_to_dps(gyro_raw, (float *) gyro_dps, GYRO_FSR);

    float gyro_rps[3];
    for (int i = 0; i < 3; i++) {
        gyro_rps[i] = gyro_dps[i] / 180.0f * (float) (M_PI);
    }

    Quaternion q = MadgwickAHRSupdateIMU(gyro_rps[0], gyro_rps[1], gyro_rps[2], accel_gs[0], accel_gs[1], accel_gs[2]);
    float quat[4] = {q.q0, q.q1, q.q2, q.q3};
    quat_to_angles(quat, (float *) angles);

    rotation_current= {angles[0], angles[2], -angles[1]};
}

Rotation MPU6050Driver::get_rotation() {
    return rotation_current;
}

Vector3 MPU6050Driver::get_acceleration() {
    auto [x, y, z] = acceleration_current;
    const float threshold = 0.1;
    x = abs(x) > threshold ? x : 0;
    y = abs(y) > threshold ? y : 0;
    z = abs(z) > threshold ? z : 0;
//    Vector3 acc_global = body_to_earth({x,y,z}, rotation_current);
    return {x,y,z} ;
//    return acc_global;
}

void MPU6050Driver::update(float dt) {

}
