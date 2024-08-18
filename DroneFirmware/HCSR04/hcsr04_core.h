#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum HCSR04_State {
    HCSR04_READY,
    HCSR04_WAITING_ECHO,
    HCSR04_MEASURING_ECHO,
    HCSR04_ERROR
} HCSR04_State;

typedef struct HCSR04_Ctx {
    HCSR04_State state;
    float distance;
} HCSR04_Ctx;

void HCSR04_init(HCSR04_Ctx* ctx);

int HCSR04_start_measurement(HCSR04_Ctx* ctx);

HCSR04_State HCSR04_get_state(HCSR04_Ctx* ctx);
float HCSR04_get_float(HCSR04_Ctx* ctx);

void HCSR04_handle_echo_rising(HCSR04_Ctx* ctx);
void HCSR04_handle_echo_falling(HCSR04_Ctx* ctx);

#ifdef __cplusplus
}
#endif