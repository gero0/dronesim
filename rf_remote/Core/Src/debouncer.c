#include "debouncer.h"

const int press_interval = 10;
const int release_interval = 100;

void reset_counter(Debouncer *db, bool state){
    db->counter = state ? (release_interval / db->check_interval) : (press_interval / db->check_interval);
}


Debouncer db_init (GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin, bool invert, int interval_ms) {
    Debouncer db = {
        .counter=0, .debounced_state = invert, .GPIO_Port= GPIO_Port,
        .GPIO_Pin = GPIO_Pin, .check_interval = interval_ms, .inverted = invert
    };
    reset_counter(&db, invert);

    return db;
}

bool db_is_pressed(Debouncer *db) {
    return db->debounced_state;
}

bool db_state_changed(Debouncer* db){
    bool state = db->state_changed;
    db->state_changed = false;
    return state;
}

void db_update(Debouncer *db) {
    bool raw_state = HAL_GPIO_ReadPin(db->GPIO_Port, db->GPIO_Pin);
    if(db->inverted){
        raw_state = !raw_state;
    }

    if(raw_state == db->debounced_state){
        reset_counter(db, raw_state);
    }else{
        db->counter--;
        if(db->counter <= 0){
            db->debounced_state = raw_state;
            reset_counter(db, raw_state);
            db->state_changed = true;
        }
    }
}

