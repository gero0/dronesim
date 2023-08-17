//
// Created by gero on 8/6/23.
//

#ifndef MPUTEST_MPU_INTERFACE_H
#define MPUTEST_MPU_INTERFACE_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <inv_mpu.h>
#include "stm32h7xx_hal.h"

void mpu_interface_register(I2C_HandleTypeDef *i2c);

unsigned long f103_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);

unsigned long f103_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);

void f103_delay_ms(unsigned long ms);

unsigned long f103_get_ms();

#ifdef __cplusplus
}
#endif

#endif //MPUTEST_MPU_INTERFACE_H
