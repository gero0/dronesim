/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void rtos_delay(uint32_t ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOC
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define NRF24_CSN_Pin GPIO_PIN_4
#define NRF24_CSN_GPIO_Port GPIOA
#define NRF24_CE_Pin GPIO_PIN_0
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_IRQ_Pin GPIO_PIN_10
#define NRF24_IRQ_GPIO_Port GPIOB
#define NRF24_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define MPU_IRQ_Pin GPIO_PIN_12
#define MPU_IRQ_GPIO_Port GPIOB
#define MPU_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define QMC_IRQ_Pin GPIO_PIN_13
#define QMC_IRQ_GPIO_Port GPIOB
#define QMC_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define VL5_IRQ_Pin GPIO_PIN_15
#define VL5_IRQ_GPIO_Port GPIOB
#define VL5_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR1_OUT_Pin GPIO_PIN_8
#define MOTOR1_OUT_GPIO_Port GPIOA
#define MOTOR2_OUT_Pin GPIO_PIN_9
#define MOTOR2_OUT_GPIO_Port GPIOA
#define MOTOR3_OUT_Pin GPIO_PIN_10
#define MOTOR3_OUT_GPIO_Port GPIOA
#define MOTOR4_OUT_Pin GPIO_PIN_11
#define MOTOR4_OUT_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_8
#define LD3_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_9
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
