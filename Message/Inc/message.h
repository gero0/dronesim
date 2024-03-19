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

typedef enum MessageType {
    Input = 0,
    DataRequest = 1,
    SetTuningsPR = 2,
    SetTuningsYT = 3,
    GetPosition = 4,
    GetStatus = 5,
    GetTuningsPR = 6,
    GetTuningsYT = 7,
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