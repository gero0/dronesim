//
// Created by gero on 10/11/23.
//

#ifndef DRONEFIRMWARE_BME_INTERFACE_H
#define DRONEFIRMWARE_BME_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bme280_defs.h>
#include <stdbool.h>
#include <stm32h7xx_hal.h>
#include <main.h>

bool bme280_i2c_init(I2C_HandleTypeDef *hi2c);

void bme280_delay_us(long unsigned int us, void *intf_ptr);

BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);


#ifdef __cplusplus
}
#endif

#endif //DRONEFIRMWARE_BME_INTERFACE_H
