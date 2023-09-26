/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FreeRTOS.h>
#include <semphr.h>
#include "task.h"
#include <cstdio>
#include <memory.h>
#include <AHRS.h>
#include <cmath>
#include "MotorDriverMock.h"
#include "DroneController.h"
#include "FreeRTOSConfig.h"
#include "nRF24_Defs.h"
#include "message.h"
#include "qmc5883l.h"

extern "C" {
#include "nRF24.h"
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
AHRS ahrs;
MotorDriverMock motor_mocks[4];
DroneController controller(&motor_mocks[0], &motor_mocks[1], &motor_mocks[2], &motor_mocks[3], &ahrs);
SemaphoreHandle_t controller_mutex;
volatile uint8_t nrf24_rx_flag, nrf24_tx_flag, nrf24_mr_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    nRF24_spi_rx_irq();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    nRF24_spi_tx_irq();
}

void rtos_delay(uint32_t Time) {
    vTaskDelay(Time / portTICK_RATE_MS);
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

bool waitTXTimeout(uint32_t timeout) {
    auto begin = xTaskGetTickCount();
    auto now = begin;
    do {
        now = xTaskGetTickCount();
        if (nRF24_TXDone()) {
            return true;
        }
    } while ( (now - begin) < timeout);
    return false;
}

int _write(int file, char *ptr, int len) {
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        __io_putchar(*ptr++);
    }
    return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Check which version of the timer triggered this callback and toggle LED
    if (htim == &htim3) {
        ahrs.sensor_update();
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

void ControlTask(void *pvParameters) {
    vTaskDelay(500 / portTICK_RATE_MS);
    bool ok = ahrs.init_hardware(&hi2c1, &hi2c1, &hi2c1);
    if (!ok) {
        printf("Could not initialize MPU!\r\n");
        while (true) {}
        //emergencystop()
    }
    ok = ahrs.calibrate();
    if (!ok) {
        printf("MPU Calibration Error!\r\n");
        while (true) {}
        //emergencystop()
    }

    HAL_TIM_Base_Start_IT(&htim3);

    uint32_t prev_time = xTaskGetTickCount();
    while (1) {
        vTaskDelay(1 / portTICK_RATE_MS);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        uint32_t now = xTaskGetTickCount();
        float dt = (float) (now - prev_time) / 1000.0f;
        xSemaphoreTake(controller_mutex, portMAX_DELAY);
        controller.update(dt);
        prev_time = now;
        xSemaphoreGive(controller_mutex);
    }

    vTaskDelete(NULL);
}

enum class CommState {
    Init,
    Listen,
    Send,
    ConnLost,
    Error
};

bool init_transceiver() {
    vTaskDelay(100 / portTICK_RATE_MS);
    nRF24_Init(&hspi1);
    vTaskDelay(100 / portTICK_RATE_MS);
    nRF24_SetCRCLength(NRF24_CRC_WIDTH_2B);
    nRF24_SetRXAddress(0, (uint8_t *) "Dro");
    nRF24_SetTXAddress((uint8_t *) "Pil");
    nRF24_RX_Mode();
    return true;
}

CommState receive_message(Message *output_msg, TickType_t *last_contact_time) {
    const TickType_t connlost_threshold = 3000;
    const float altitude_const = 0.1;
    const float max_angle = (35.0f / 180.0f) * M_PI;
    const float yaw_constant = 0.1f;

    static TickType_t last_angle_input;
    static TickType_t last_altitude_input;

    xSemaphoreTake(controller_mutex, portMAX_DELAY);

    auto angles = controller.get_rotation();
    auto pos = controller.get_position();
    auto press_altitude = controller.get_altitude();
    auto radar_altitude = controller.get_radar_altitude();

    xSemaphoreGive(controller_mutex);

    uint8_t usb_buffer[64];
    usb_buffer[0] = 11;
    usb_buffer[1] = 37;
    memcpy(usb_buffer + 2, &angles, sizeof(angles));
    memcpy(usb_buffer + 14, &pos, sizeof(pos));
    memcpy(usb_buffer + 26, &press_altitude, sizeof(press_altitude));
    memcpy(usb_buffer + 30, &radar_altitude, sizeof(radar_altitude));
    HAL_UART_Transmit(&huart3, usb_buffer, 64, HAL_MAX_DELAY);

    uint8_t input_buffer[NRF24_PAYLOAD_SIZE];

//    if (nRF24_RXAvailible()) {
//        *last_contact_time = xTaskGetTickCount();
//        uint8_t read;
//        nRF24_ReadRXPaylaod(input_buffer, &read);
//        Message msg = parse_message(input_buffer);
//        switch (msg.type) {
//            case AnglesInput: {
//                float pitch_input = *(float *) (&msg.data[0]);
//                float yaw_input = *(float *) (&msg.data[sizeof(float)]);
//                float roll_input = *(float *) (&msg.data[2 * sizeof(float)]);
//
//                pitch_input = std::clamp(pitch_input, -1.0f, 1.0f);
//                yaw_input = std::clamp(yaw_input, -1.0f, 1.0f);
//                roll_input = std::clamp(roll_input, -1.0f, 1.0f);
//
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller.set_pitch(pitch_input * max_angle);
//                controller.set_roll(roll_input * max_angle);
//
//                auto timestamp = xTaskGetTickCount();
//                if (timestamp > last_angle_input) {
//                    last_angle_input = timestamp;
//                    auto [pitch, yaw, roll] = controller.get_rotation_setpoints();
//                    yaw = yaw + yaw_input * yaw_constant;
//                    controller.set_yaw(yaw);
//                }
//                xSemaphoreGive(controller_mutex);
//                return CommState::Listen;
//            }
//            case AltitudeInput: {
//                float alt_input = *(float *) (&msg.data[0]);
//                auto timestamp = xTaskGetTickCount();
//                if (timestamp > last_altitude_input) {
//                    last_altitude_input = timestamp;
//                    float altitude_sp = controller.get_altitude_setpoint();
//                    altitude_sp += alt_input * altitude_const;
//                    xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                    controller.set_altitude(altitude_sp);
//                    xSemaphoreGive(controller_mutex);
//                    return CommState::Listen;
//                }
//            }
//                break;
//            case HoldCommand:
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller.hover();
//                xSemaphoreGive(controller_mutex);
//                return CommState::Listen;
//            case RTOCommand:
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                controller.RTO();
//                xSemaphoreGive(controller_mutex);
//                return CommState::Listen;
//            case GetAngles: {
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                Rotation rot = controller.get_rotation();
//                xSemaphoreGive(controller_mutex);
//                output_msg->type = GetAngles;
//                memcpy(output_msg->data, &rot, sizeof(rot));
//                return CommState::Send;
//            }
//            case GetPosition: {
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                Vector3 pos = controller.get_position();
//                xSemaphoreGive(controller_mutex);
//                output_msg->type = GetPosition;
//                memcpy(output_msg->data, &pos, sizeof(pos));
//                return CommState::Send;
//            }
//            case GetAltitude: {
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                float alt = controller.get_altitude();
//                float radar = controller.get_radar_altitude();
//                xSemaphoreGive(controller_mutex);
//                output_msg->type = GetAltitude;
//                memcpy(output_msg->data, &alt, sizeof(float));
//                memcpy(output_msg->data + sizeof(float), &radar, sizeof(float));
//                return CommState::Send;
//            }
//            case GetStatus: {
//                xSemaphoreTake(controller_mutex, portMAX_DELAY);
//                auto speeds = controller.get_motor_speeds();
//                xSemaphoreGive(controller_mutex);
//                uint8_t speeds_int[4];
//                for (int i = 0; i < 4; i++) {
//                    speeds_int[i] = static_cast<uint8_t>(speeds[i] * 100.0f);
//                }
//                output_msg->type = GetStatus;
//                memcpy(output_msg->data, speeds_int, sizeof(uint8_t) * 4);
//                return CommState::Send;
//            }
//        }
//    }
//    if (xTaskGetTickCount() - *last_contact_time > connlost_threshold) {
//        return CommState::ConnLost;
//    }
    return CommState::Listen;
}

void CommTask(void *pvParameters) {
    CommState comm_state = CommState::Init;
    Vector3 position_of_last_contact = {0.0f, 0.0f, 0.0f};
    Message output_msg;

    TickType_t last_contact_time = xTaskGetTickCount();

    while (1) {
        switch (comm_state) {
            case CommState::Init: {
                bool ok = init_transceiver();
                if (ok) {
                    comm_state = CommState::Listen;
                } else {
                    comm_state = CommState::Error;
                }
            }
                break;

            case CommState::Listen:
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, static_cast<GPIO_PinState>(false));
                comm_state = receive_message(&output_msg, &last_contact_time);
                break;

            case CommState::Send: {
                HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, static_cast<GPIO_PinState>(true));
                uint8_t output_buffer[NRF24_PAYLOAD_SIZE];
                nRF24_TX_Mode();
                serialize_msg(output_buffer, &output_msg);
                nRF24_WriteTXPayload(output_buffer, NRF24_PAYLOAD_SIZE);
                waitTXTimeout(10);
                nRF24_WaitTX();
                nRF24_RX_Mode();
                comm_state = CommState::Listen;
            }
                break;

            case CommState::ConnLost:
                xSemaphoreTake(controller_mutex, portMAX_DELAY);
                controller.hover();
                controller.set_hover_setpoint(position_of_last_contact);
                xSemaphoreGive(controller_mutex);
                break;

            case CommState::Error:
                //emergencystop()
                break;
        }
    }

    vTaskDelete(NULL);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    controller_mutex = xSemaphoreCreateMutex();
    xTaskCreate(CommTask, "CommTask", 200, NULL, 1, NULL);
    xTaskCreate(ControlTask, "ControlTask", 600, NULL, 2, NULL);
    vTaskStartScheduler();

    while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20 -1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
