//
// Created by gero on 10/11/23.
//

#include "bme_interface.h"

static uint8_t dev_addr = (0x76 << 1);
static I2C_HandleTypeDef *i2c = NULL;

BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(i2c, dev_addr, reg_addr, 1, reg_data, length, 2);
    if (res != HAL_OK)
        return BME280_E_COMM_FAIL;

    return BME280_INTF_RET_SUCCESS;
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(i2c, dev_addr, reg_addr, 1, reg_data, length, 2);
    if (res != HAL_OK)
        return BME280_E_COMM_FAIL;

    return BME280_INTF_RET_SUCCESS;
}

bool bme280_i2c_init(I2C_HandleTypeDef *hi2c) {
    i2c = hi2c;
    return true;
}

//TODO: replace with timer based approach?
void bme280_delay_us(long unsigned int us, void *intf_ptr) {
    for (long unsigned int i = 0; i < us * 100; i++) {
        __NOP();
    }
}
