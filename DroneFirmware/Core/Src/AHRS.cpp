//
// Created by gero on 8/9/23.
//

#include <cmath>
#include <vl53l0x_platform.h>
#include "AHRS.h"

#define GYRO_FSR 1000
#define ACC_FSR 8

extern "C" {
#include "inv_mpu.h"
}

#include "mpu_interface.h"
#include "mpu_helpers.h"
#include "qmc5883l.h"
#include "main.h"
#include "bme_interface.h"

constexpr int AHRS_SAMPLE_RATE = 500;

bool AHRS::init_hardware(I2C_HandleTypeDef *mpu_i2c, I2C_HandleTypeDef *qmc_i2c, I2C_HandleTypeDef *vl5_i2c,
                         I2C_HandleTypeDef *bme_i2c) {
    mpu_interface_register(mpu_i2c);
    struct int_param_s int_param;
    bool result = (mpu_init(&int_param) == 0);
    if (!result) {
        return false;
    }
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_accel_fsr(ACC_FSR);
    mpu_set_gyro_fsr(GYRO_FSR);
    mpu_set_sample_rate(200);

    result = qmc_init(qmc_i2c);
    if (!result) {
        return false;
    }

    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * AHRS_SAMPLE_RATE, /* 5 seconds */
    };

    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &settings);

    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    Dev->I2cDevAddr = 0x52;
    Dev->I2cHandle = vl5_i2c;

    VL53L0X_WaitDeviceBooted(Dev);
    VL53L0X_DataInit(Dev);
    VL53L0X_StaticInit(Dev);
    VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    VL53L0X_StartMeasurement(Dev);

    int8_t rslt;
    bme280_i2c_init(bme_i2c);

    bme_dev.read = bme280_i2c_read;
    bme_dev.write = bme280_i2c_write;
    bme_dev.intf = BME280_I2C_INTF;
    bme_dev.delay_us = bme280_delay_us;

    rslt = bme280_init(&bme_dev);
    result = (rslt == 0);
    if (!result) {
        return false;
    }

    bme280_get_sensor_settings(&bme_settings, &bme_dev);
    bme_settings.filter = BME280_FILTER_COEFF_16;
    bme_settings.osr_h = BME280_OVERSAMPLING_1X;
    bme_settings.osr_p = BME280_OVERSAMPLING_4X;
    bme_settings.osr_t = BME280_OVERSAMPLING_1X;
    bme_settings.standby_time = BME280_STANDBY_TIME_0_5_MS;
    bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &bme_settings, &bme_dev);
    bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme_dev);
    bme280_cal_meas_delay(&bme_period, &bme_settings);

    initialized = true;

    return true;
}


bool AHRS::calibrate() const {
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

static bme280_data get_pressure(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    uint8_t status_reg;
    bme280_data comp_data;

    rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

    if (status_reg & BME280_STATUS_MEAS_DONE)
    {
            /* Measurement time delay given to read sample */
//            dev->delay_us(period, dev->intf_ptr);

        rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, dev);
    }

    return comp_data;
}

void AHRS::madgwick_update() {
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 1.0 / AHRS_SAMPLE_RATE);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    rotation_current = {(float) (euler.angle.pitch / 180.0f * M_PI), (float) (euler.angle.yaw / 180.0f * M_PI),
                        (float) (euler.angle.roll / 180.0f * M_PI)};
}

Rotation AHRS::get_rotation() {
    return rotation_current;
}

Vector3 AHRS::get_acceleration() {
    auto [x, y, z] = acceleration_current;
    const float threshold = 0.1;
    x = abs(x) > threshold ? x : 0;
    y = abs(y) > threshold ? y : 0;
    z = abs(z) > threshold ? z : 0;
//    Vector3 acc_global = body_to_earth({x,y,z}, rotation_current);
    return {x, y, z};
//    return acc_global;
}

void AHRS::update(float dt) {
    //TODO: data ready check
    if (!initialized) {
        return;
    }
    short accel_raw[3];
    short gyro_raw[3];
    float accel_gs[3];

    mpu_get_accel_reg(accel_raw, nullptr);
    accel_to_gs(accel_raw, (float *) accel_gs, ACC_FSR);

//    acc_samples[acc_i] = {accel_gs[1], accel_gs[0], -accel_gs[2]};
//    acc_i = (acc_i + 1) % averaging_len;
//    acceleration_current = {0, 0, 0};
//    for (auto acc_sample: acc_samples) {
//        acceleration_current += acc_sample;
//    }
//    acceleration_current /= averaging_len;

    mpu_get_gyro_reg(gyro_raw, nullptr);
    gyro_to_dps(gyro_raw, (float *) gyro_dps, GYRO_FSR);

    __disable_irq();
    accelerometer = {-accel_gs[1], accel_gs[0], accel_gs[2]};
    gyroscope = {-gyro_dps[1], gyro_dps[0], gyro_dps[2]};
    __enable_irq();

    if (qmc_data_ready()) {
        __disable_irq();
        magnetometer = {static_cast<float>(-qmc_get_y()), static_cast<float>(qmc_get_x()),
                        static_cast<float>(qmc_get_z())};
        __enable_irq();
    }

    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    uint8_t vl5_dataready = 0;
    VL53L0X_GetMeasurementDataReady(Dev, &vl5_dataready);
    if (vl5_dataready == 0x01) {
        VL53L0X_GetRangingMeasurementData(Dev, &RangingMeasurementData);
        VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        radar_altitude = static_cast<float>(RangingMeasurementData.RangeMilliMeter) / 1000.0f;
    }

    double pressure = get_pressure(bme_period, &bme_dev).pressure;
    //international barometric formula
    //https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702
    double alt = 44330.0 * (1 - std::pow((pressure/101325),(1/5.255)) );
    altitude = static_cast<float>(alt);
}


float AHRS::get_altitude() {
    return altitude;
}

float AHRS::get_radar_altitude() {
    return radar_altitude;
}
