//
// Created by gero on 9/17/23.
//

#include "qmc5883l.h"
#include "main.h"

I2C_HandleTypeDef *i2c = NULL;

void qmc_write_register(uint16_t memaddr, uint8_t *data, uint16_t length) {
    if (!i2c) {
        return;
    }
    HAL_I2C_Mem_Write(i2c, (QMC_ADDR << 1), memaddr, 1, data, length, HAL_MAX_DELAY);
}

void qmc_read_register(uint16_t memaddr, uint8_t *data, uint16_t length) {
    if (!i2c) {
        return;
    }
    HAL_I2C_Mem_Read(i2c, (QMC_ADDR << 1) | 1, memaddr, 1, data, length, HAL_MAX_DELAY);
}

void qmc_delay_ms(uint16_t ms){
    rtos_delay(ms);
}

bool qmc_init(I2C_HandleTypeDef *hi2c) {
    i2c = hi2c;
    qmc_set_reset_period();
    qmc_delay_ms(1);
    qmc_set_oversampling(QMC_NumOfSamples_512);
    qmc_delay_ms(1);
    qmc_set_output_rate(QMC_DataOutputRate_200);
    qmc_delay_ms(1);
    qmc_set_range(QMC_Range_8G);
    qmc_delay_ms(1);
    qmc_set_operating_mode(QMC_OperatingMode_Continuous);
    qmc_set_interrupt_enabled(true);
    qmc_set_ptr_rollover_enabled(false);
    qmc_delay_ms(6);
    uint8_t buf;
    qmc_read_register(QMC_CR1, &buf, 1);
    return true;
}

int16_t read_data_output(uint8_t reg_msb, uint8_t reg_lsb) {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    qmc_read_register(reg_lsb, &lsb, 1);
    qmc_read_register(reg_msb, &msb, 1);
    return (int16_t) (lsb) | ((int16_t) (msb) << 8);
}

int16_t qmc_get_x() {
    return read_data_output(QMC_DATA_X_MSB, QMC_DATA_X_LSB);
}

int16_t qmc_get_y() {
    return read_data_output(QMC_DATA_Y_MSB, QMC_DATA_Y_LSB);
}

int16_t qmc_get_z() {
    return read_data_output(QMC_DATA_Z_MSB, QMC_DATA_Z_LSB);
}

bool qmc_data_ready() {
    uint8_t status = 0;
    qmc_read_register(QMC_STATUS, &status, 1);
    return (bool) (status & 0b1);
}

bool qmc_data_overflow() {
    uint8_t status = 0;
    qmc_read_register(QMC_STATUS, &status, 1);
    return (bool) (status & 0b10);
}

bool qmc_data_skip() {
    uint8_t status = 0;
    qmc_read_register(QMC_STATUS, &status, 1);
    return (bool) (status & 0b100);
}

int16_t read_temperature(){
    return read_data_output(QMC_TEMP_MSB, QMC_TEMP_LSB);
}

void qmc_set_operating_mode(QMC_OperatingMode opmode) {
    uint8_t cr1 = 0;
    qmc_read_register(QMC_CR1, &cr1, 1);
    cr1 &= ~0b00000011;
    cr1 |= (uint8_t) (opmode);
    qmc_write_register(QMC_CR1, &cr1, 1);
}

void qmc_set_output_rate(QMC_DataOutputRate dr) {
    uint8_t cr1 = 0;
    qmc_read_register(QMC_CR1, &cr1, 1);
    cr1 &= ~0b00001100;
    cr1 |= ((uint8_t) (dr) << 2);
    qmc_write_register(QMC_CR1, &cr1, 1);
}

void qmc_set_range(QMC_Range range) {
    uint8_t cr1 = 0;
    qmc_read_register(QMC_CR1, &cr1, 1);
    cr1 &= ~0b00110000;
    cr1 |= (uint8_t) (range << 4);
    qmc_write_register(QMC_CR1, &cr1, 1);
}

void qmc_set_oversampling(QMC_NumOfSamples samples) {
    uint8_t cr1 = 0;
    qmc_read_register(QMC_CR1, &cr1, 1);
    cr1 &= ~0b11000000;
    cr1 |= (uint8_t) (samples << 6);
    qmc_write_register(QMC_CR1, &cr1, 1);
}

void qmc_set_interrupt_enabled(bool enabled) {
    uint8_t cr2 = 0;
    qmc_read_register(QMC_CR2, &cr2, 1);
    if(enabled){
        cr2 &= ~(0b1);
    }else{
        cr2 |= 0b1;
    }
    qmc_write_register(QMC_CR2, &cr2, 1);
}

void qmc_set_ptr_rollover_enabled(bool enabled) {
    uint8_t cr2 = 0;
    qmc_read_register(QMC_CR2, &cr2, 1);
    if(enabled){
        cr2 |= (1 << 6);
    }else{
        cr2 &= ~(1 << 6);
    }
    qmc_write_register(QMC_CR2, &cr2, 1);
}

void qmc_soft_reset() {
    uint8_t cr2 = (1 << 7);
    qmc_write_register(QMC_CR2, &cr2, 1);
}

void qmc_set_reset_period() {
    uint8_t sr = 0x01;
    qmc_write_register(QMC_SET_RESET, &sr, 1);
}











