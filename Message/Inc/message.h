#ifndef MESSAGE_MESSAGE_H
#define MESSAGE_MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>
#define MSG_DATA_SIZE 31

typedef enum MessageType {
    AnglesInput = 0,
    AltitudeInput = 1,
    LandCommand = 2,
    EStopCommand = 3,
    HoldCommand = 4,
    RTOCommand = 5,
    GetAngles = 6,
    GetPosition = 7,
    GetAltitude = 8,
    GetStatus = 9,
    GetSetpoints = 10,
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