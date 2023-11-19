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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JOY0_X_Pin GPIO_PIN_0
#define JOY0_X_GPIO_Port GPIOA
#define JOY0_Y_Pin GPIO_PIN_1
#define JOY0_Y_GPIO_Port GPIOA
#define JOY1_X_Pin GPIO_PIN_2
#define JOY1_X_GPIO_Port GPIOA
#define JOY1_Y_Pin GPIO_PIN_3
#define JOY1_Y_GPIO_Port GPIOA
#define JOY1_SW_Pin GPIO_PIN_4
#define JOY1_SW_GPIO_Port GPIOA
#define JOY0_SW_Pin GPIO_PIN_5
#define JOY0_SW_GPIO_Port GPIOA
#define LED_AUX_0_Pin GPIO_PIN_6
#define LED_AUX_0_GPIO_Port GPIOA
#define LED_AUX_1_Pin GPIO_PIN_7
#define LED_AUX_1_GPIO_Port GPIOA
#define BTN_0_Pin GPIO_PIN_0
#define BTN_0_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_1
#define BTN_1_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_8
#define LCD_RW_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_GPIO_Port GPIOA
#define BTN_2_Pin GPIO_PIN_10
#define BTN_2_GPIO_Port GPIOA
#define BTN_3_Pin GPIO_PIN_11
#define BTN_3_GPIO_Port GPIOA
#define GPIO_NRF_CSN_Pin GPIO_PIN_12
#define GPIO_NRF_CSN_GPIO_Port GPIOA
#define GPIO_NRF_CE_Pin GPIO_PIN_15
#define GPIO_NRF_CE_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_9
#define LCD_D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
