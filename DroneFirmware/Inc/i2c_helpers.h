//
// Created by gero on 11/28/23.
//

#ifndef I2C_HELPERS_H
#define I2C_HELPERS_H

#ifdef __cplusplus
extern "C"{
#endif


HAL_StatusTypeDef i2c_transmit_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                   uint16_t MemAddSize, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef i2c_receive_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                  uint16_t MemAddSize, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef i2c_master_transmit_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size);

HAL_StatusTypeDef i2c_master_receive_dma(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint8_t* pData, uint16_t Size);

void i2c_tx_complete_callback();

void i2c_rx_complete_callback();

#ifdef __cplusplus
    }
#endif

#endif //I2C_HELPERS_H
