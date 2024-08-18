#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

static uint32_t timestamp = 0;

void HCSR04_delay_us(uint16_t us)
{
    // assuming timer is running at 1MHz
    uint32_t start = TIM11->CNT;
    while (true) {
        uint32_t current = TIM11->CNT;
        if (current < start) {
            current += 65535;
        }
        if (current >= start + us) {
            return;
        }
    }
}

void HCSR04_write_trig_pin(bool state)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, state);
}

void HCSR04_start_timer()
{
    timestamp = TIM11->CNT;
}

int64_t HCSR04_stop_timer()
{
    uint32_t current = TIM11->CNT;
    if (current < timestamp) {
        current += 65535;
    }
    return current - timestamp;
}