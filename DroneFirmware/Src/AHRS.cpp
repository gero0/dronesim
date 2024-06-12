//
// Created by gero on 8/9/23.
//

#include <cmath>
#include "AHRS.h"

#define GYRO_FSR 1000
#define ACC_FSR 8

extern "C" {
#include "inv_mpu.h"
}

#include "mpu_interface.h"
#include "mpu_helpers.h"
#include "qmc5883l.h"
#include "bme_interface.h"
#include "task.h"
#include "hcsr04_core.h"

constexpr int AHRS_SAMPLE_RATE = 993;

bool AHRS::init_hardware(I2C_HandleTypeDef *mpu_i2c, I2C_HandleTypeDef *qmc_i2c,
                         I2C_HandleTypeDef *bme_i2c) {
    bool result = init_mpu(mpu_i2c);
    result &= qmc_init(qmc_i2c);

    init_fusion();
    HCSR04_init(&hc_driver);

    result &= init_bme(bme_i2c);

    initialized = result;

    arm_fir_init_f32(&gx_fir, fir_length, &firCoeff[0], &gx_fir_state[0], fir_block_size);
    arm_fir_init_f32(&gy_fir, fir_length, &firCoeff[0], &gy_fir_state[0], fir_block_size);
    arm_fir_init_f32(&gz_fir, fir_length, &firCoeff[0], &gz_fir_state[0], fir_block_size);
    arm_fir_init_f32(&ax_fir, fir_length, &firCoeff[0], &ax_fir_state[0], fir_block_size);
    arm_fir_init_f32(&ay_fir, fir_length, &firCoeff[0], &ay_fir_state[0], fir_block_size);
    arm_fir_init_f32(&az_fir, fir_length, &firCoeff[0], &az_fir_state[0], fir_block_size);

    return initialized;
}

bool AHRS::init_mpu(I2C_HandleTypeDef *mpu_i2c) {
    mpu_interface_register(mpu_i2c);
    int_param_s int_param;
    if (const bool result = mpu_init(&int_param) == 0; !result) {
        return false;
    }
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_accel_fsr(ACC_FSR);
    mpu_set_gyro_fsr(GYRO_FSR);
    mpu_set_sample_rate(1000);
    return true;
}

void AHRS::init_fusion() {
    constexpr FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 1000,
            .accelerationRejection = 10.0f,
//            .magneticRejection = 10.0f,
//        .recoveryTriggerPeriod = 1 * AHRS_SAMPLE_RATE, /* 5 seconds */
            .recoveryTriggerPeriod = 0 /* 5 seconds */
    };

    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &settings);
}

bool AHRS::init_bme(I2C_HandleTypeDef *bme_i2c) {
    bme280_i2c_init(bme_i2c);

    bme_dev.read = bme280_i2c_read;
    bme_dev.write = bme280_i2c_write;
    bme_dev.intf = BME280_I2C_INTF;
    bme_dev.delay_us = bme280_delay_us;

    const int8_t rslt = bme280_init(&bme_dev);
    if (const bool result = rslt == 0; !result) {
        return false;
    }

    bme280_get_sensor_settings(&bme_settings, &bme_dev);
    bme_settings.filter = BME280_FILTER_COEFF_16;
    bme_settings.osr_h = BME280_OVERSAMPLING_1X;
    bme_settings.osr_p = BME280_OVERSAMPLING_16X;
    bme_settings.osr_t = BME280_OVERSAMPLING_2X;
    bme_settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
    bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &bme_settings, &bme_dev);
    bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme_dev);
    bme280_cal_meas_delay(&bme_period, &bme_settings);

    bme_timestamp = xTaskGetTickCount();

    std::fill(alt_samples, alt_samples + num_alt_samples, 0.0);

    return true;
}

bool AHRS::calibrate() {
    if (!initialized) {
        return false;
    }

    if (const int result = mpu_run_self_test(gyro_bias, accel_bias); result != 0x7) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = static_cast<long>(static_cast<float>(gyro_bias[i]) * 32.8f); //convert to +-1000dps
        accel_bias[i] *= 2048.f; //convert to +-16G
        accel_bias[i] = accel_bias[i] >> 16;
        gyro_bias[i] = gyro_bias[i] >> 16;
    }

    mpu_set_gyro_bias_reg(gyro_bias);
    mpu_set_accel_bias_6050_reg(accel_bias);

    return true;
}

bme280_data AHRS::get_pressure(uint32_t period, bme280_dev *dev) {
    uint8_t status_reg;
    bme280_data comp_data{};

    int8_t rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, dev);

    return comp_data;
}

void AHRS::madgwick_update(float dt) {
    FusionVector gyro_filtered;
    FusionVector acc_filtered;

    arm_fir_f32(&gx_fir, &gyroscope.array[0], &gyro_filtered.array[0], fir_block_size);
    arm_fir_f32(&gy_fir, &gyroscope.array[1], &gyro_filtered.array[1], fir_block_size);
    arm_fir_f32(&gz_fir, &gyroscope.array[2], &gyro_filtered.array[2], fir_block_size);
    arm_fir_f32(&ax_fir, &accelerometer.array[0], &acc_filtered.array[0], fir_block_size);
    arm_fir_f32(&ay_fir, &accelerometer.array[1], &acc_filtered.array[1], fir_block_size);
    arm_fir_f32(&az_fir, &accelerometer.array[2], &acc_filtered.array[2], fir_block_size);

    gyroscope = gyro_filtered;
    accelerometer = acc_filtered;

    angular_rate = {gyroscope.axis.y, gyroscope.axis.z, gyroscope.axis.x};

    if(!mag_broken){
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    }
    else{
        //failsafe when QMC randomly stops working
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    }

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    rotation_current = {
            static_cast<float>(euler.angle.pitch / 180.0f * M_PI), static_cast<float>(euler.angle.yaw / 180.0f * M_PI),
            static_cast<float>(euler.angle.roll / 180.0f * M_PI)
    };
}

Rotation AHRS::get_rotation() {
    return rotation_current;
}

Rotation AHRS::get_angular_rate() {
    return angular_rate;
}

Vector3 AHRS::get_acceleration() {
    auto [x, y, z] = acceleration_current;
    constexpr float threshold = 0.1;
    x = abs(x) > threshold ? x : 0;
    y = abs(y) > threshold ? y : 0;
    z = abs(z) > threshold ? z : 0;
    return {x, y, z};
}

void AHRS::update(float dt) {
    if (!initialized) {
        return;
    }
    short accel_raw[3];
    short gyro_raw[3];
    float accel_gs[3];

    mpu_get_accel_reg(accel_raw, nullptr);
    accel_to_gs(accel_raw, accel_gs, ACC_FSR);

    mpu_get_gyro_reg(gyro_raw, nullptr);
    gyro_to_dps(gyro_raw, const_cast<float *>(gyro_dps), GYRO_FSR);

    accelerometer = {-accel_gs[0], -accel_gs[1], accel_gs[2]};
    gyroscope = {-gyro_dps[0], -gyro_dps[1], gyro_dps[2]};

    size_t current_time = xTaskGetTickCount();

    if(qmc_dataready){
        int16_t temp[3];
        qmc_read_all_axes(temp);
        float x_ut = ((float) temp[0] / 32767.0f) * 8.0f * 100.0f;
        float y_ut = ((float) temp[1] / 32767.0f) * 8.0f * 100.0f;
        float z_ut = ((float) temp[2] / 32767.0f) * 8.0f * 100.0f;

        float X = x_ut - mag_bias[0];
        float Y = y_ut - mag_bias[1];
        float Z = z_ut - mag_bias[2];

        float X_corr = mag_matrix[0][0] * X + mag_matrix[0][1] * Y + mag_matrix[0][2] * Z;
        float Y_corr = mag_matrix[1][0] * X + mag_matrix[1][1] * Y + mag_matrix[1][2] * Z;
        float Z_corr = mag_matrix[2][0] * X + mag_matrix[2][1] * Y + mag_matrix[2][2] * Z;

        magnetometer = {
                -Y_corr, X_corr, Z_corr
        };

        qmc_dataready = false;
        qmc_timestamp = current_time;
    }

    if(current_time - qmc_timestamp >= 1000){
        mag_broken = true;
    }

    madgwick_update(dt);
}


float AHRS::get_altitude() {
    return altitude;
}

float AHRS::get_radar_altitude() {
    return radar_altitude;
}

float AHRS::get_absolute_altitude() {
    return abs_altitude;
}

float AHRS::get_vs(){
    return vertical_speed;
}

void AHRS::qmc_ready() {
    qmc_dataready = true;
}

void AHRS::altitude_update(SemaphoreHandle_t controller_mutex) {
    if(!initialized){
        return;
    }
    size_t current_time = xTaskGetTickCount();

    if (current_time - bme_timestamp >= (bme_period / 1000)) {
        const double pressure = get_pressure(bme_period, &bme_dev).pressure;
        const double alt = 44330.0 * (1 - std::pow(pressure / 101325.0, 1 / 5.255));
        auto new_altitude = static_cast<float>(alt);

        xSemaphoreTake(controller_mutex, portMAX_DELAY);
        altitude = abs_altitude - base_altitude;
        vertical_speed = (new_altitude - abs_altitude) / (0.1f);
        abs_altitude = new_altitude;
        bme_timestamp = current_time;
        xSemaphoreGive(controller_mutex);
    }

    if (current_time - last_hc_measurement_ts > 150) {
        last_hc_measurement_ts = current_time;
        HCSR04_init(&hc_driver);
        return;
    }

    if (HCSR04_get_state(&hc_driver) == HCSR04_ERROR) {
        HCSR04_init(&hc_driver);
        return;
    }

    if (HCSR04_get_state(&hc_driver) != HCSR04_READY) {
        return;
    }

    float dist = HCSR04_get_float(&hc_driver);
    HCSR04_start_measurement(&hc_driver);
    last_hc_measurement_ts = current_time;

    bool valid_radar = (dist >= 0.0 && dist <= 1.2);

    xSemaphoreTake(controller_mutex, portMAX_DELAY);

    radar_altitude = dist;

    if(radar_altitude < 0.1f && valid_radar){
        altitude = radar_altitude;
        base_altitude = abs_altitude - altitude;
    }
    else if (radar_altitude < 0.8f && valid_radar) {
        altitude = radar_altitude * abs(cos(rotation_current.pitch) * cos(rotation_current.roll));
        base_altitude = abs_altitude - altitude;
    }
    xSemaphoreGive(controller_mutex);
}

void AHRS::hc_isr() {
    if (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)) {
        HCSR04_handle_echo_rising(&hc_driver);
    } else {
        HCSR04_handle_echo_falling(&hc_driver);
    }
}
