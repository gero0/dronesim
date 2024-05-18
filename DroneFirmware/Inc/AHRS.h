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

    Rotation get_angular_rate() override;

    Vector3 get_acceleration() override;

    float get_vs() override;

    float get_altitude() override;

    float get_radar_altitude() override;

    float get_absolute_altitude() override;

    void update(float dt) override;

    void vl5_ready();

    void qmc_ready();

private:
    bool initialized = false;
    volatile float gyro_dps[3];
    const float g_const = 9.81;

    const signed char orientation[9] = {1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1};

    static const int averaging_len = 64;
    Vector3 acceleration_current{};

    Rotation rotation_current{};
    Rotation angular_rate{0.0f, 0.0f, 0.0f};

    FusionVector accelerometer{.0f, .0f, .0f};
    FusionVector gyroscope{.0f, .0f, .0f};
    FusionVector magnetometer{.0f, .0f, .0f};
    FusionAhrs ahrs{};

    VL53L0X_Dev_t vl53l0x_c{}; // center module
    VL53L0X_DEV Dev = &vl53l0x_c;

    uint32_t bme_period = 0;
    size_t bme_timestamp = 0;
    bme280_dev bme_dev{};
    bme280_settings bme_settings{};

    float altitude = 0.0f;
    float vertical_speed = 0.0f;
    float abs_altitude = 0.0f;
    float base_altitude = 0.0f;
    float radar_altitude = 0.0f;

    long accel_bias[3];
    long gyro_bias[3];

    bool init_mpu(I2C_HandleTypeDef *mpu_i2c);

    void init_fusion();

    void init_vl5(I2C_HandleTypeDef *vl5_i2c) const;

    bool init_bme(I2C_HandleTypeDef *bme_i2c);

    bme280_data get_pressure(uint32_t period, bme280_dev *dev);

    static constexpr int num_alt_samples = 4;
    double alt_samples[num_alt_samples];

    volatile bool vl5_dataready = false;
    volatile bool qmc_dataready = false;

    static constexpr int fir_block_size = 1;
    static constexpr int fir_length = 64;

    //FS = 993Hz, FC=20HZ FIR LPF
    float32_t firCoeff[fir_length] = {
            -0.0006441707702596293, -0.0006021449189092179, -0.000571487607069365, -0.0005360928947526213,
            -0.00047564286102582266, -0.0003662810145966364, -0.0001814979266249004, 0.00010680188364875711,
            0.0005270917988435571, 0.0011069211575542516, 0.0018715725624149412, 0.002842734891923391,
            0.004037249494516885, 0.00546598561199099, 0.0071328972059114155, 0.009034307170505148,
            0.011158456636983343, 0.01348534702985119, 0.015986891129009067, 0.018627377089989995, 0.02136423669043036,
            0.024149096538044656, 0.026929079127321173, 0.02964830997808964, 0.0322495780917248, 0.03467609001574128,
            0.03687325322594478, 0.03879042252949271, 0.040382543866083684, 0.04161163322842563, 0.042448034313893775,
            0.0428714067249028, 0.0428714067249028, 0.042448034313893775, 0.04161163322842563, 0.040382543866083684,
            0.03879042252949271, 0.03687325322594478, 0.03467609001574128, 0.0322495780917248, 0.029648309978089638,
            0.02692907912732117, 0.024149096538044642, 0.02136423669043036, 0.018627377089989988, 0.015986891129009067,
            0.013485347029851184, 0.011158456636983343, 0.009034307170505145, 0.0071328972059114095,
            0.005465985611990989, 0.004037249494516881, 0.002842734891923391, 0.0018715725624149397,
            0.0011069211575542516, 0.0005270917988435567, 0.00010680188364875697, -0.0001814979266249002,
            -0.0003662810145966359, -0.00047564286102582266, -0.0005360928947526213, -0.000571487607069365,
            -0.0006021449189092179, -0.0006441707702596293,
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
