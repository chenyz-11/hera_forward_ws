//
// Created by lewis on 2021/10/29.
//

#ifndef DXFDSIMULATION_DXFD_MSG_FILE_OPERATOR_H
#define DXFDSIMULATION_DXFD_MSG_FILE_OPERATOR_H

#include <iostream>
#include "protocol.h"

#define DXFD_MSG_ID_FILE_OPERATOR_LEN 255

enum OperationType{
    UPLOAD = 0,
    DOWNLOAD,
    SELECTED,
    DELETEO,
    NOTIFY,
} ;

enum FileType{
    XML_FILE,
    CSV_FILE,
    FILE_LIST,
    UART_IDENTIFIER,
    WARNING,
    ERRORO,
};

#pragma pack(push,1)
typedef struct __dxfd_file_operator_t {
    uint8_t seq;
    uint8_t operationType;
    uint8_t fileType;
    uint8_t filename[DXFD_MSG_ID_FILE_OPERATOR_LEN - 2];
} dxfd_file_operator_t;
#pragma pack(pop)

static inline uint16_t dxfd_msg_file_operator_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                    uint8_t seq, uint8_t operationType, uint8_t fileType,
                                                    char* filename, uint16_t length = 0)
{
    if(length > DXFD_MSG_ID_FILE_OPERATOR_LEN - 2)
    {
        std::cerr << "out size of dxfd_file_operator msg" << std::endl;
        return 0;
    }
    dxfd_file_operator_t packet;
    packet.seq = seq;
    packet.operationType = operationType;
    packet.fileType = fileType;
    memcpy(packet.filename, filename, length);
    length = length + sizeof(packet.operationType) + sizeof(fileType);
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, length);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_FILE_OPERATOR;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline uint16_t dxfd_msg_file_operator_encode(dxfd_message_t* msg, uint8_t upperID, uint16_t lowerID,
                                                    dxfd_file_operator_t packet, uint16_t length = 0)
{
    if(length > DXFD_MSG_ID_FILE_OPERATOR_LEN - 2)
    {
        std::cerr << "out size of dxfd_file_operator msg" << std::endl;
        return 0;
    }
    memcpy(_DXFD_PAYLOAD_NON_CONST(msg), &packet, length);
    msg->head = DXFD_PROTOCOL_HEAD;
    msg->msgID = DXFD_MSG_ID_FILE_OPERATOR;
    msg->lowerID = lowerID;
    msg->upperID = upperID;
    msg->length = length;
    msg->checksum = calculateChecksum(msg);
    return length + DXFD_PROTOCOL_LEN;
}

static inline void dxfd_msg_file_operator_decode(const dxfd_message_t* msg,
                                                 dxfd_file_operator_t* file_operator)
{
    int len = msg->length < DXFD_MSG_ID_FILE_OPERATOR_LEN ? msg->length : DXFD_MSG_ID_FILE_OPERATOR_LEN;
    memset(file_operator, 0, DXFD_MSG_ID_FILE_OPERATOR_LEN);
    memcpy(file_operator, &msg->payload[0], len);
}

#endif //DXFDSIMULATION_DXFD_MSG_FILE_OPERATOR_H
