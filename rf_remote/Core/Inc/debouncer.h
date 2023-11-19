#ifndef RF_REMOTE_DEBOUNCER_H
#define RF_REMOTE_DEBOUNCER_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct Debouncer {
    bool debounced_state;
    int counter;
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
    int check_interval;
    bool inverted;
    bool state_changed;
} Debouncer;

Debouncer db_init(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, bool invert, int interval_ms);
bool db_is_pressed(Debouncer* db);
bool db_state_changed(Debouncer* db);
void db_update(Debouncer *db);

#endif //RF_REMOTE_DEBOUNCER_H
