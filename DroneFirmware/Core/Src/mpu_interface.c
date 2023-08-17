//
// Created by gero on 8/6/23.
//

#include "../Inc/mpu_interface.h"

I2C_HandleTypeDef* hi2c;

void mpu_interface_register(I2C_HandleTypeDef *i2c) {
    hi2c = i2c;
}

unsigned long f103_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data){
    addr = addr << 1;
    if(HAL_I2C_Mem_Write(hi2c, (uint16_t)addr, (uint16_t)reg, 1, data, len, 100) != HAL_OK){
        return 1;
    }
    return 0;
}

unsigned long f103_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data){
    addr = addr << 1;
    if(HAL_I2C_Mem_Read(hi2c, (uint16_t)addr, (uint16_t)reg, 1, data, len, 100) != HAL_OK){
        return 1;
    }
    return 0;
}

void f103_delay_ms(unsigned long ms){
    HAL_Delay(ms);
}

unsigned long f103_get_ms(unsigned long ms){
    return HAL_GetTick();
}

int reg_int_cb(struct int_param_s *int_param)
{
    return 0;
}
