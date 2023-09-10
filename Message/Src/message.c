#include <memory.h>
#include "../Inc/message.h"

Message parse_message(uint8_t buffer[]) {
    Message msg;
    msg.type = (MessageType)(buffer[0]);
    memcpy(msg.data, &buffer[1], MSG_DATA_SIZE);
    return msg;
}

void serialize_msg(uint8_t *buffer, const Message *msg) {
    buffer[0] = (uint8_t) msg->type;
    memcpy(buffer + 1, msg->data, MSG_DATA_SIZE);
}
