#ifndef TYPEDEF_H
#define TYPEDEF_H

/**
 * @brief Typedef for CAN message
 *
 * This structure is used to represent a CAN message. It contains the
 * identifier of the message, the data bytes and the DLC (data length
 * code) of the message.
 *
 * @struct CAN_Msg_t
 * @var Identifier The identifier of the message
 * @var Data The data bytes of the message
 * @var DLC The DLC (data length code) of the message
 */
typedef struct {
    uint32_t Identifier;
    uint8_t  Data[8];
    uint8_t  DLC;
} CAN_Msg_t;

#endif
