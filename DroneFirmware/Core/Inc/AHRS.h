//
// Created by gero on 8/9/23.
//

#ifndef DRONEFIRMWARE_AHRS_H
#define DRONEFIRMWARE_AHRS_H

#include "stm32h7xx_hal.h"
#include "algebra.h"
#include "SensorReader.h"
#include <Fusion.h>
#include "vl53l0x_api.h"
#include "bme280.h"

class AHRS : public SensorReader {
public:
    bool init_hardware(I2C_HandleTypeDef *mpu_i2c, I2C_HandleTypeDef *qmc_i2c,
                       I2C_HandleTypeDef *vl5_i2c, I2C_HandleTypeDef *bme_i2c);

    bool calibrate() const;

    void sensor_update();

    Rotation get_rotation() override;

    Vector3 get_acceleration() override;

    virtual float get_altitude() override;

    virtual float get_radar_altitude() override;

    void update(float dt) override;

private:
    bool initialized = false;
    volatile float gyro_dps[3];
    const float g_const = 9.81;

    const signed char orientation[9] = {1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1};

    static const int averaging_len = 64;
    Vector3 acceleration_current;
    Vector3 acc_samples[averaging_len];
    int acc_i = 0;

    Rotation rotation_current;

    FusionVector accelerometer{.0f, .0f, .0f};
    FusionVector gyroscope{.0f, .0f, .0f};
    FusionVector magnetometer{.0f, .0f, .0f};
    FusionAhrs ahrs;

    VL53L0X_Dev_t vl53l0x_c; // center module
    VL53L0X_DEV Dev = &vl53l0x_c;

    uint32_t bme_period = 0;
    bme280_dev bme_dev;
    bme280_settings bme_settings;

    float altitude = 0.0f;
    float radar_altitude = 0.0f;
};

#endif //DRONEFIRMWARE_AHRS_H
