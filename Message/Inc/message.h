#ifndef MESSAGE_MESSAGE_H
#define MESSAGE_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>
#define MSG_DATA_SIZE 31

#define MSG_LAND_CMD 1
#define MSG_ESTOP_CMD 2
#define MSG_HOLD_CMD 4
#define MSG_RTO_CMD 8
#define MSG_STOP_CMD 16

typedef enum MessageType {
    Input = 0,
    DataRequest = 1,
    SetTuningsPR = 2,
    SetTuningsYA = 3,
    SetTuningsPrRr = 4,
    SetTuningsYrVs = 5,
    GetPosition = 6,
    GetStatus = 7,
    GetRates = 8,
    GetTuningsPR = 9,
    GetTuningsYA = 10,
    GetTuningsPrRr = 11,
    GetTuningsYrVs = 12,
} MessageType;

typedef struct Message {
    MessageType type;
    uint8_t data[MSG_DATA_SIZE];
} Message;

Message parse_message(uint8_t buffer[]);
void serialize_msg(uint8_t *buffer, const Message *msg);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MESSAGE_MESSAGE_H