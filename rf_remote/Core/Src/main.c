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
#include <memory.h>
#include <stdio.h>
#include "nRF24.h"
#include "nRF24_Defs.h"
#include "queue.h"
#include "message.h"
#include "lcd.h"
#include "debouncer.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Debouncer sw1_debouncer;

volatile uint8_t nrf24_rx_flag, nrf24_tx_flag, nrf24_mr_flag;

size_t last_input_update = 0;
size_t last_response_time = 0;
size_t last_msg_time = 0;
MessageType last_response_type = 0;
MessageType last_msg_type = 0;

float press_altitude = 0.0f;
float radar_altitude = 0.0f;
float abs_altitude = 0.0f;

//pitch, yaw, roll
float angles[3] = {0};
//X Y Z (global)
float pos[3] = {0};

uint8_t motors[4] = {0};

float joy0_x = 0.0f;
float joy0_y = 0.0f;
float joy1_x = 0.0f;
float joy1_y = 0.0f;

float pitch_sp = 0.0f;
float yaw_sp = 0.0f;
float roll_sp = 0.0f;
float altitude_sp = 0.0f;

uint16_t adc_readings[4];

bool connection_ok = false;

//int __io_putchar(int ch) {
////    CDC_Transmit_FS((uint8_t *) &ch, 1);
//    return ch;
//}

//int _write(int file, char *ptr, int len) {
//    int DataIdx;
//
//    for (DataIdx = 0; DataIdx < len; DataIdx++) {
//        __io_putchar(*ptr++);
//    }
//    return len;
//}

float to_degrees(float a){
    return a / 3.14f * 180.0f;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim4){
        db_update(&sw1_debouncer);
    }

}

void check_inputs() {
    joy0_x = (float) (adc_readings[0]) / (4096);
    joy0_y = (float) (adc_readings[1]) / (4096);
    joy1_x = (float) (adc_readings[2]) / (4096);
    joy1_y = (float) (adc_readings[3]) / (4096);
}

typedef enum DisplayScreen{
    DisplayScreen_Summary = 0,
    DisplayScreen_Analog = 1,
    DisplayScreen_Rotation = 2,
    DisplayScreen_Position = 3,
    DisplayScreen_Motors = 4,
    DisplayScreen_Altitude = 5,
    DisplayScreen_Other = 6,
    DisplayScreen_Setpoints = 7,
} DisplayScreen;

typedef void (*Display_t)(char[], char[]);

void display_draw_summary(char line_1[], char line_2[]){
    char is_connected = connection_ok ? '^' : '!';
    //TODO: replace with status received from drone
    const char* status = "OK";
    float yaw = angles[1];
    sprintf(line_1, "%c HDG: %.0f %s", is_connected, to_degrees(yaw), status);
    sprintf(line_2, "ALT: %.1f (%.1f)", press_altitude, radar_altitude);
}

void display_draw_analog(char line_1[], char line_2[]){
    sprintf(line_1, "X:%.3f|X:%.3f", joy1_x, joy0_x);
    sprintf(line_2, "Y:%.3f|Y:%.3f", joy1_y, joy0_y);
}

void display_draw_rotation(char line_1[], char line_2[]){
    sprintf(line_1, "P:%.2f R:%.2f", to_degrees(angles[0]), to_degrees(angles[2]));
    sprintf(line_2, "Y:%.2f", to_degrees(angles[1]));
}

void display_draw_position(char line_1[], char line_2[]){
    sprintf(line_1, "X:%.2f Y:%.2f", pos[0], pos[1]);
}

void display_draw_motors(char line_1[], char line_2[]){
    sprintf(line_1, "M1: %d M2: %d", motors[0], motors[1]);
    sprintf(line_2, "M3: %d M4: %d", motors[2], motors[3]);
}

void display_draw_altitude(char line_1[], char line_2[]){
    sprintf(line_1, "R:%0.1f A:%0.1f", press_altitude, abs_altitude);
    sprintf(line_2, "RDR:%0.2f", radar_altitude);
}

void display_draw_other(char line_1[], char line_2[]){
    sprintf(line_1, "LRT:");
    sprintf(line_2, "%d", last_response_time);
}

void display_draw_setpoints(char line_1[], char line_2[]){
    sprintf(line_1, "%.2f %.2f", pitch_sp, roll_sp);
    sprintf(line_2, "%.2f %.2f", yaw_sp, altitude_sp);
}

Display_t screens[] = {
        display_draw_summary,
        display_draw_analog,
        display_draw_rotation,
        display_draw_position,
        display_draw_motors,
        display_draw_altitude,
        display_draw_other,
        display_draw_setpoints,
};

DisplayScreen currentScreen = DisplayScreen_Summary;

void update_display(){
    LCD_clear();
    char buffer1[32] = {0};
    char buffer2[32] = {0};
    //get text for currently selected screen
    screens[currentScreen](buffer1, buffer2);
    LCD_position(1,1);
    LCD_write_text(buffer1, strlen(buffer1));
    LCD_position(1,2);
    LCD_write_text(buffer2, strlen(buffer2));
}

void update_values(Message msg) {
    switch (msg.type) {
        case GetAltitude:
            memcpy(&press_altitude, msg.data, sizeof(float));
            memcpy(&radar_altitude, &msg.data[sizeof(float)], sizeof(float));
            memcpy(&abs_altitude, &msg.data[2*sizeof(float)], sizeof(float));
            break;
        case GetAngles:
            memcpy(angles, msg.data, sizeof(float) * 3);
            break;
        case GetPosition:
            memcpy(pos, msg.data, sizeof(float) * 3);
            break;
        case GetStatus:
            memcpy(motors, msg.data, sizeof(uint8_t) * 4);
            break;
        case GetSetpoints:
            memcpy(&pitch_sp, &msg.data[0], sizeof(float));
            memcpy(&yaw_sp, &msg.data[4], sizeof(float));
            memcpy(&roll_sp, &msg.data[8], sizeof(float));
            memcpy(&altitude_sp, &msg.data[12], sizeof(float));
        default:
            break;
    }
}

void transmit_message(Message msg) {
    uint8_t buffer[32];
    serialize_msg(buffer, &msg);
    nRF24_WriteTXPayload(buffer, NRF24_PAYLOAD_SIZE);
    nRF24_WaitTX();
}

Message receive_message() {
    uint8_t buffer[32] = {0};
    uint8_t read = 0;
    nRF24_ReadRXPaylaod(buffer, &read);

    Message resp;
    resp.type = buffer[0];
    memcpy(resp.data, &buffer[1], 31);
    last_response_type = resp.type;

    return resp;
}

void init_transceiver(){
    nRF24_Init(&hspi2);
    nRF24_SetRXAddress(0, (uint8_t *) "Pil");
    nRF24_SetTXAddress((uint8_t *) "Dro");
    nRF24_SetCRCLength(NRF24_CRC_WIDTH_2B);
    nRF24_TX_Mode();
}

void send_message(Queue *queue) {
    const int response_tries_treshold = 50;

    if (!queue_empty(queue)) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
        Message *msg = queue_get(queue);

        transmit_message(*msg);
        bool response_received = false;

        for (int i = 0; i < response_tries_treshold && !response_received; i++) {
            HAL_Delay(1);
            response_received = nRF24_RXAvailible();
        }

        if (response_received) {
            Message resp = receive_message();
            connection_ok = true;
            last_response_time = HAL_GetTick();
            update_values(resp);
        }else{
            connection_ok = false;
            nRF24_FlushTX();
            nRF24_FlushRX();
            init_transceiver();
            nRF24_FlushTX();
            nRF24_FlushRX();
        }

        last_msg_time = HAL_GetTick();
        last_msg_type = msg->type;
        queue_pop(queue);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
    }
}

void delay_us(uint16_t us)
{
    HAL_TIM_Base_Stop(&htim3);
    htim3.Instance->CNT = 0;
    HAL_TIM_Base_Start(&htim3);
    while (htim3.Instance->CNT <= us) {
        //wait
    }
}

Message create_angles_msg(){
    Message msg;
    msg.type = AnglesInput;
//    float yaw = 0.f;
    float yaw = 0.983f - 2.0f * joy1_y;
    float pitch = 0.983f - 2.0f * joy0_y;
    float roll = 2.0f * joy0_x - 0.983f;
    memcpy(&msg.data[0], &pitch, 4);
    memcpy(&msg.data[4], &yaw, 4);
    memcpy(&msg.data[8], &roll, 4);
    return msg;
}

Message create_altitude_msg(){
    Message msg;
    msg.type = AltitudeInput;
    float alt = 2.0f * joy1_x - 0.983f;
    memcpy(&msg.data[0], &alt, 4);
    return msg;
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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    sw1_debouncer = db_init(JOY1_SW_GPIO_Port, JOY1_SW_Pin, true, 5);
    
    HAL_Delay(1000);

    init_transceiver();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

    Queue *msg_queue = queue_create(100, sizeof(Message));

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_readings, 4);

    //debouncing timer
    HAL_TIM_Base_Start_IT(&htim4);

    //Init LCD display
    LCD_init(delay_us);
    LCD_clear();
    LCD_write_text("oh no", 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//    MessageType mt = GetAngles;
    MessageType mt = AnglesInput;

    const int msg_send_period = 80;
    const int screen_update_period = 50;
    size_t last_screen_update = 0;

    while (1) {
        check_inputs();
        size_t current_time = HAL_GetTick();

        if(current_time - last_screen_update >= screen_update_period){
            update_display();
            last_screen_update = HAL_GetTick();
        }

        if (current_time - last_msg_time > msg_send_period) {
            switch (mt) {
                case AnglesInput:{
                    Message msg = create_angles_msg();
                    queue_push(msg_queue, &msg);
                }
                    break;
                case AltitudeInput: {
                    Message msg = create_altitude_msg();
                    queue_push(msg_queue, &msg);
                }
                    break;
                case HoldCommand:
                    //TODO
                    break;
                case RTOCommand:
                    //TODO
                    break;
                default:
                    break;
            }
            mt++;
            if (mt > AltitudeInput){
                mt = AnglesInput;
            }
            send_message(msg_queue);
        }

        if (db_is_pressed(&sw1_debouncer) && db_state_changed(&sw1_debouncer)){
            currentScreen += 1;
            if(currentScreen > DisplayScreen_Setpoints){
                currentScreen = DisplayScreen_Summary;
            }
        }
    }

    //debug usb printing
//        uint8_t usb_buffer[64];
//        usb_buffer[0] = 11;
//        usb_buffer[1] = 37;
//        memcpy(usb_buffer + 2, angles, sizeof(angles));
//        memcpy(usb_buffer + 14, pos, sizeof(pos));
//        memcpy(usb_buffer + 26, &press_altitude, sizeof(press_altitude));
//        memcpy(usb_buffer + 30, &radar_altitude, sizeof(radar_altitude));
//        memcpy(usb_buffer + 34, &motors, sizeof(motors));
//        uint32_t last_message_t = (uint32_t)last_msg_type;
//        uint32_t last_response_t = (uint32_t)last_response_type;
//        memcpy(usb_buffer + 38, &last_message_t, sizeof(last_message_t));
//        memcpy(usb_buffer + 42, &last_response_t, sizeof(last_response_t));
//        memcpy(usb_buffer + 46, &last_msg_time, sizeof(last_msg_time));
//        memcpy(usb_buffer + 50, &last_response_time, sizeof(last_response_time));
//        memcpy(usb_buffer + 54, &last_input_update, sizeof(last_input_update));
//        memcpy(usb_buffer + 58, &current_time, sizeof(current_time));
//        CDC_Transmit_FS(usb_buffer, 64);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 36-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_AUX_0_Pin|LED_AUX_1_Pin|LCD_RW_Pin|LCD_E_Pin
                          |GPIO_NRF_CSN_Pin|GPIO_NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY1_SW_Pin JOY0_SW_Pin */
  GPIO_InitStruct.Pin = JOY1_SW_Pin|JOY0_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_AUX_0_Pin LED_AUX_1_Pin LCD_RW_Pin LCD_E_Pin
                           GPIO_NRF_CSN_Pin GPIO_NRF_CE_Pin */
  GPIO_InitStruct.Pin = LED_AUX_0_Pin|LED_AUX_1_Pin|LCD_RW_Pin|LCD_E_Pin
                          |GPIO_NRF_CSN_Pin|GPIO_NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_0_Pin BTN_1_Pin */
  GPIO_InitStruct.Pin = BTN_0_Pin|BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_2_Pin BTN_3_Pin */
  GPIO_InitStruct.Pin = BTN_2_Pin|BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin
                           LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin
                          |LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
