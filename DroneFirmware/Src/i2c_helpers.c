//
// Created by gero on 11/28/23.
//

#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include "main.h"
#include "i2c_helpers.h"

volatile static bool finished = false;
volatile static bool error = false;

HAL_StatusTypeDef i2c_transmit_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t* pData, uint16_t Size) {
    error = false;
    finished = false;
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
    while (!finished && !error) {
        rtos_delay(1);
    }
    return result;
}

HAL_StatusTypeDef i2c_receive_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                  uint16_t MemAddSize, uint8_t* pData, uint16_t Size) {
    error = false;
    finished = false;
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
    while (!finished && !error) {
        rtos_delay(1);
    }
    return result;
}

HAL_StatusTypeDef i2c_master_transmit_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size) {
    error = false;
//    finished = false;
//    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, pData, Size);
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Size, 2);
//    while (!finished && !error) {
//        rtos_delay(1);
//    }
    return result;
}

HAL_StatusTypeDef i2c_master_receive_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size) {
    error = false;
//    finished = false;
//    HAL_StatusTypeDef result = HAL_I2C_Master_Receive_DMA(hi2c, DevAddress, pData, Size);
    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, Size, 2);
//    while (!finished && !error) {
//        rtos_delay(1);
//    }
    return result;
}

void i2c_tx_complete_callback() {
    finished = true;
}

void i2c_rx_complete_callback() {
    finished = true;
}

void i2c_error_callback() {
    error = true;
}



