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
#include "FreeRTOS.h"
#include "semphr.h"

class AHRS : public SensorReader {
public:
    bool init_hardware(I2C_HandleTypeDef *mpu_i2c, I2C_HandleTypeDef *qmc_i2c,
                       I2C_HandleTypeDef *vl5_i2c, I2C_HandleTypeDef *bme_i2c);

    bool calibrate();

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

    void altitude_update(SemaphoreHandle_t controller_mutex);

private:
    void madgwick_update(float dt);

    bool initialized = false;
    volatile float gyro_dps[3];

    Vector3 acceleration_current{};

    Rotation rotation_current{};
    Rotation angular_rate{0.0f, 0.0f, 0.0f};

    FusionVector accelerometer{.0f, .0f, .0f};
    FusionVector gyroscope{.0f, .0f, .0f};
    FusionVector magnetometer{.0f, .0f, .0f};
    FusionAhrs ahrs{};

    VL53L0X_Dev_t vl53l0x_c{};
    VL53L0X_DEV Dev = &vl53l0x_c;

    uint32_t bme_period = 0;
    size_t bme_timestamp = 0;
    size_t qmc_timestamp = 0;
    size_t vl5_timestamp = 0;
    bme280_dev bme_dev{};
    bme280_settings bme_settings{};

    float altitude = 0.0f;
    float vertical_speed = 0.0f;
    float abs_altitude = 0.0f;
    float base_altitude = 0.0f;
    float radar_altitude = 0.0f;

    long accel_bias[3];
    long gyro_bias[3];

    //For those sensors that like to stop cooperating sometimes for no reason
    bool mag_broken = false;
    bool vl5_broken = false;

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
//    static constexpr int fir_length = 64;
    static constexpr int fir_length = 32;

    //FS = 993Hz, FC=20HZ FIR LPF
//    float32_t firCoeff[fir_length] = {
//            -0.0006441707702596293, -0.0006021449189092179, -0.000571487607069365, -0.0005360928947526213,
//            -0.00047564286102582266, -0.0003662810145966364, -0.0001814979266249004, 0.00010680188364875711,
//            0.0005270917988435571, 0.0011069211575542516, 0.0018715725624149412, 0.002842734891923391,
//            0.004037249494516885, 0.00546598561199099, 0.0071328972059114155, 0.009034307170505148,
//            0.011158456636983343, 0.01348534702985119, 0.015986891129009067, 0.018627377089989995, 0.02136423669043036,
//            0.024149096538044656, 0.026929079127321173, 0.02964830997808964, 0.0322495780917248, 0.03467609001574128,
//            0.03687325322594478, 0.03879042252949271, 0.040382543866083684, 0.04161163322842563, 0.042448034313893775,
//            0.0428714067249028, 0.0428714067249028, 0.042448034313893775, 0.04161163322842563, 0.040382543866083684,
//            0.03879042252949271, 0.03687325322594478, 0.03467609001574128, 0.0322495780917248, 0.029648309978089638,
//            0.02692907912732117, 0.024149096538044642, 0.02136423669043036, 0.018627377089989988, 0.015986891129009067,
//            0.013485347029851184, 0.011158456636983343, 0.009034307170505145, 0.0071328972059114095,
//            0.005465985611990989, 0.004037249494516881, 0.002842734891923391, 0.0018715725624149397,
//            0.0011069211575542516, 0.0005270917988435567, 0.00010680188364875697, -0.0001814979266249002,
//            -0.0003662810145966359, -0.00047564286102582266, -0.0005360928947526213, -0.000571487607069365,
//            -0.0006021449189092179, -0.0006441707702596293,
//    };
    float32_t firCoeff[fir_length] = {
            0.002484477921641184, 0.003098999325658232, 0.004479906737840293, 0.006765294571123855,
            0.010031322764461066, 0.01428087522631932, 0.01943785657013609, 0.025347822154755437, 0.03178510297174657,
            0.038466027768688855, 0.04506730978839878, 0.051248202948630306, 0.05667468451446184, 0.06104372072526709,
            0.06410563783762427, 0.06568275817324684, 0.06568275817324684, 0.06410563783762427, 0.06104372072526709,
            0.05667468451446184, 0.0512482029486303, 0.04506730978839876, 0.038466027768688855, 0.03178510297174656,
            0.02534782215475543, 0.019437856570136084, 0.01428087522631932, 0.010031322764461066, 0.006765294571123855,
            0.004479906737840293, 0.003098999325658232, 0.002484477921641184,
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

    static constexpr float mag_matrix[3][3]  = {
            {1.253260, -0.020579, -0.004206},
            {-0.020579, 1.197955, 0.014074},
            {-0.004206, 0.014074, 1.326982},
    };
    static constexpr float mag_bias[3] = {0.734107, 5.501961, -2.091495};
};

#endif //DRONEFIRMWARE_AHRS_H
