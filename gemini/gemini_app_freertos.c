/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "canbus_project.h" // Assuming this contains your structs and IDs
#include "fdcan.h"
#include "crc.h"
#include "rtc.h"

/* Private Variables ---------------------------------------------------------*/
struct canNodeInfo nodeInfo;
uint8_t introMsgPtr = 0;
uint8_t introMsgCnt = 0;
bool FLAG_BEGIN_NORMAL_OPER = false;
bool FLAG_SEND_INTRODUCTION = false;

/* External Handles from main.c */
extern osMessageQId canTxQueueHandle;
extern uint32_t board_crc; // Calculated in Init

/* --- Refactored Introduction Logic --- */
static void txIntroduction(int ptr) {
    CAN_Msg_t newMsg; // Temporary struct to push to Queue
    
    if (ptr <= 0) {
        newMsg.Identifier = nodeInfo.nodeType;
        // Map nodeInfo.nodeID to newMsg.Data...
        memcpy(newMsg.Data, nodeInfo.nodeID, 4);
        osMessagePut(canTxQueueHandle, (uint32_t)&newMsg, 0);
    } 
    // ... Implement remaining module ptr logic similarly using osMessagePut
}

/* --- Refactored CAN RX Task --- */
void StartCanRxTask(void const * argument) {
    for(;;) {
        // Wait for notification from the ISR (FDCAN callback)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            
            // 1. Extract Remote ID (First 4 bytes of payload)
            uint32_t msgRemoteId = (rxData[0]) | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);
            bool isForMe = (msgRemoteId == board_crc);

            // 2. Handle Global and Specific Commands
            switch (rxHeader.Identifier) {
                case MSG_NORM_OPER:
                    FLAG_BEGIN_NORMAL_OPER = true;
                    break;
                
                case MSG_REQ_INTRO:
                    introMsgPtr = 0;
                    FLAG_SEND_INTRODUCTION = true;
                    break;

                case ACK_INTRO:
                    if (isForMe && (introMsgPtr < introMsgCnt)) {
                        introMsgPtr++;
                        FLAG_SEND_INTRODUCTION = true;
                    }
                    break;

                case SW_SET_ON:
                    if (isForMe) {
                        // rxOutputState(rxData[4], OUT_STATE_ON);
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
                    }
                    break;
                
                // Add remaining cases from your Arduino switch...
            }
        }
    }
}

/* --- Refactored Status/Telemetry Task --- */
// Replacing the Arduino HardwareTimer "nodeCheckStatus"
void StartDefaultTask(void const * argument) {
    // Initialization: Set up nodeInfo based on board_crc
    memcpy(nodeInfo.nodeID, &board_crc, 4);
    nodeInfo.nodeType = BOX_MULTI_IO; // Example
    introMsgCnt = 4; 

    for(;;) {
        // 1. Handle Introduction State Machine
        if (FLAG_SEND_INTRODUCTION) {
            txIntroduction(introMsgPtr);
            if (introMsgPtr >= introMsgCnt) {
                FLAG_SEND_INTRODUCTION = false;
            }
        }

        // 2. Handle Periodic Telemetry (Normal Operation)
        if (FLAG_BEGIN_NORMAL_OPER) {
            // Push sensor data to canTxQueueHandle
            // Example:
            // CAN_Msg_t tempMsg = { .Identifier = DATA_NODE_CPU_TEMP, .Data = ... };
            // osMessagePut(canTxQueueHandle, (uint32_t)&tempMsg, 10);
        }

        osDelay(1000); // Replaces TRANSMIT_RATE_MS
    }
}