#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>
#define MSG_DATA_SIZE 31

typedef enum MessageType {
    AnglesInput = 0,
    AltitudeInput = 1,
    HoldCommand = 2,
    RTOCommand = 3,
    GetAngles = 4,
    GetPosition = 5,
    GetAltitude = 6,
    GetStatus = 7,
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