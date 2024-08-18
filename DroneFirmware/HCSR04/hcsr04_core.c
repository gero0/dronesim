#include <stdbool.h>
#include <stdint.h>

#include "hcsr04_core.h"

extern void HCSR04_delay_us(int us);
extern void HCSR04_write_trig_pin(bool state);
extern void HCSR04_start_timer();
extern int64_t HCSR04_stop_timer();

int HCSR04_start_measurement(HCSR04_Ctx* ctx)
{
    if (ctx->state != HCSR04_READY) {
        return -1;
    }
    HCSR04_write_trig_pin(true);
    HCSR04_delay_us(10);
    HCSR04_write_trig_pin(false);

    ctx->state = HCSR04_WAITING_ECHO;

    return 0;
}

void HCSR04_init(HCSR04_Ctx* ctx)
{
    ctx->distance = -1.0f;
    ctx->state = HCSR04_READY;
}

HCSR04_State HCSR04_get_state(HCSR04_Ctx* ctx)
{
    return ctx->state;
}

float HCSR04_get_float(HCSR04_Ctx* ctx)
{
    return ctx->distance;
}

void HCSR04_handle_echo_rising(HCSR04_Ctx* ctx)
{
    if (ctx->state != HCSR04_WAITING_ECHO) {
        ctx->state = HCSR04_ERROR;
        return;
    }
    ctx->state = HCSR04_MEASURING_ECHO;
    HCSR04_start_timer();
}

void HCSR04_handle_echo_falling(HCSR04_Ctx* ctx)
{
    if (ctx->state != HCSR04_MEASURING_ECHO) {
        ctx->state = HCSR04_ERROR;
        return;
    }
    int64_t time = HCSR04_stop_timer();

    if (time < 1) {
        ctx->state = HCSR04_ERROR;
        return;
    }

    double time_s = (double)time / 1000000;
    // s = v * t
    double distance = 343.0 * time_s;

    ctx->distance = (float)distance / 2.0f;
    ctx->state = HCSR04_READY;
}
