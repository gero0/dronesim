//
// Created by gero on 8/9/23.
//

#ifndef DRONEFIRMWARE_AHRS_H
#define DRONEFIRMWARE_AHRS_H

#include "stm32f4xx_hal.h"
#include "algebra.h"
#include "SensorReader.h"
#include <Fusion.h>
#include "vl53l0x_api.h"
#include "bme280.h"
#include "arm_math.h"

class AHRS : public SensorReader {
public:
    bool init_hardware(I2C_HandleTypeDef *mpu_i2c, I2C_HandleTypeDef *qmc_i2c,
                       I2C_HandleTypeDef *vl5_i2c, I2C_HandleTypeDef *bme_i2c);

    bool calibrate();

    void madgwick_update();

    Rotation get_rotation() override;

    Vector3 get_acceleration() override;

    virtual float get_altitude() override;

    virtual float get_radar_altitude() override;

    virtual float get_absolute_altitude() override;

    void update(float dt) override;

    void vl5_ready();
private:
    bool initialized = false;
    volatile float gyro_dps[3];
    const float g_const = 9.81;

    const signed char orientation[9] = {1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1};

    static const int averaging_len = 64;
    Vector3 acceleration_current {};
    Vector3 acc_samples[averaging_len] {};
    int acc_i = 0;

    Rotation rotation_current {};

    FusionVector accelerometer{.0f, .0f, .0f};
    FusionVector gyroscope{.0f, .0f, .0f};
    FusionVector magnetometer{.0f, .0f, .0f};
    FusionAhrs ahrs {};

    VL53L0X_Dev_t vl53l0x_c {}; // center module
    VL53L0X_DEV Dev = &vl53l0x_c;

    uint32_t bme_period = 0;
    size_t bme_timestamp = 0;
    bme280_dev bme_dev {};
    bme280_settings bme_settings {};

    float altitude = 0.0f;
    float abs_altitude = 0.0f;
    float base_altitude = 0.0f;
    float radar_altitude = 0.0f;

    static bool init_mpu(I2C_HandleTypeDef *mpu_i2c);
    void init_fusion();
    void init_vl5(I2C_HandleTypeDef *vl5_i2c) const;
    bool init_bme(I2C_HandleTypeDef *bme_i2c);
    bme280_data get_pressure(uint32_t period, bme280_dev* dev);

    static constexpr int num_alt_samples = 4;
    double alt_samples[num_alt_samples];
    int alt_samples_i = 0;

    volatile bool vl5_dataready = false;

    static constexpr int fir_block_size = 1;
    static constexpr int fir_length = 32;

    //FS = 200Hz, FC=30HZ FIR LPF
    float32_t firCoeff[fir_length] = {
            0.00146026f,  0.0017447f,   0.00043153f, -0.00291798f, -0.00608365f, -0.00407316f,
            0.00576994f,  0.01735429f,  0.01686372f, -0.00500784f, -0.03819911f, -0.05165392f,
            -0.01261744f,  0.08462113f,  0.20467264f,  0.28763488f,  0.28763488f,  0.20467264f,
            0.08462113f, -0.01261744f, -0.05165392f, -0.03819911f, -0.00500784f,  0.01686372f,
            0.01735429f,  0.00576994f, -0.00407316f, -0.00608365f, -0.00291798f,  0.00043153f,
            0.0017447f,   0.00146026f,
    };

    arm_fir_instance_f32 gx_fir;
    arm_fir_instance_f32 gy_fir;
    arm_fir_instance_f32 gz_fir;
    arm_fir_instance_f32 ax_fir;
    arm_fir_instance_f32 ay_fir;
    arm_fir_instance_f32 az_fir;

    float32_t gx_fir_state[fir_block_size + fir_length - 1];
    float32_t gy_fir_state[fir_block_size + fir_length - 1];
    float32_t gz_fir_state[fir_block_size + fir_length - 1];
    float32_t ax_fir_state[fir_block_size + fir_length - 1];
    float32_t ay_fir_state[fir_block_size + fir_length - 1];
    float32_t az_fir_state[fir_block_size + fir_length - 1];
};

#endif //DRONEFIRMWARE_AHRS_H
