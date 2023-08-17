//
// Created by gero on 8/9/23.
//

#ifndef DRONEFIRMWARE_MPU6050DRIVER_H
#define DRONEFIRMWARE_MPU6050DRIVER_H

#include "stm32h7xx_hal.h"
#include "algebra.h"
#include "SensorReader.h"

class MPU6050Driver : public SensorReader{
public:
    bool init_hardware(I2C_HandleTypeDef* i2c);
    bool calibrate() const;
    void sensor_update();
    Rotation get_rotation() override;
    Vector3 get_acceleration() override;
    void update(float dt) override;
private:
    bool initialized = false;
//    volatile float accel_gs[3];
    volatile float gyro_dps[3];
//    volatile float angles[3];
    const float g_const = 9.81;

    const signed char orientation[9] = {1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1};

    const int GYRO_FSR=2000;
    const int ACC_FSR=8;

    static const int averaging_len = 64;
    Vector3 acceleration_current;
    Vector3 acc_samples[averaging_len];
    int acc_i = 0;

    Rotation rotation_current;
};

#endif //DRONEFIRMWARE_MPU6050DRIVER_H
