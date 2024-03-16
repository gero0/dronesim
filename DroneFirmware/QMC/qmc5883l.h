//
// Created by gero on 9/17/23.
//
#ifdef __cplusplus
extern "C"{
#endif

#ifndef DRONEFIRMWARE_QMC5883L_H
#define DRONEFIRMWARE_QMC5883L_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define QMC_ADDR 0x0D

#define QMC_DATA_X_LSB    0x0
#define QMC_DATA_X_MSB    0x1
#define QMC_DATA_Y_LSB    0x2
#define QMC_DATA_Y_MSB    0x3
#define QMC_DATA_Z_LSB    0x4
#define QMC_DATA_Z_MSB    0x5
#define QMC_STATUS        0x6
#define QMC_TEMP_LSB      0x7
#define QMC_TEMP_MSB      0x8
#define QMC_CR1           0x9
#define QMC_CR2           0xA
#define QMC_SET_RESET     0xB
#define QMC_ID            0xD


typedef enum QMC_NumOfSamples {
    QMC_NumOfSamples_512 = 0,
    QMC_NumOfSamples_256 = 1,
    QMC_NumOfSamples_128 = 2,
    QMC_NumOfSamples_64 = 3,
} QMC_NumOfSamples;

typedef enum QMC_DataOutputRate {
    QMC_DataOutputRate_10 = 0,
    QMC_DataOutputRate_50 = 1,
    QMC_DataOutputRate_100 = 2,
    QMC_DataOutputRate_200 = 3,
} QMC_DataOutputRate;

typedef enum QMC_Range {
    QMC_Range_2G = 0,
    QMC_Range_8G = 1,
} QMC_Range;

typedef enum QMC_OperatingMode {
    QMC_OperatingMode_Standby = 0,
    QMC_OperatingMode_Continuous = 1,
} QMC_OperatingMode;

bool qmc_init(I2C_HandleTypeDef *hi2c);

int16_t qmc_get_x();

int16_t qmc_get_y();

int16_t qmc_get_z();

void qmc_read_all_axes(int16_t out[3]);

void qmc_set_oversampling(QMC_NumOfSamples samples);

void qmc_set_output_rate(QMC_DataOutputRate dr);

void qmc_set_range(QMC_Range gain);

void qmc_set_operating_mode(QMC_OperatingMode opmode);

bool qmc_data_ready();

bool qmc_data_overflow();

bool qmc_data_skip();

void qmc_set_interrupt_enabled(bool enabled);

void qmc_set_ptr_rollover_enabled(bool enabled);

void qmc_soft_reset();

void qmc_read_register(uint16_t memaddr, uint8_t *data, uint16_t length);

void qmc_set_reset_period();

#endif //DRONEFIRMWARE_QMC5883L_H

#ifdef __cplusplus
}
#endif