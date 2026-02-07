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

void updatePhysicalOutput(uint8_t index, uint8_t state) {
    GPIO_PinState pinState = (state == OUT_STATE_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    
    switch(index) {
        case 0: HAL_GPIO_WritePin(OUT_0_GPIO_Port, OUT_0_Pin, pinState); break;
        case 1: HAL_GPIO_WritePin(OUT_1_GPIO_Port, OUT_1_Pin, pinState); break;
        // ... and so on
    }
}
void nodeCheckStatus(void) {
    if (FLAG_SEND_INTRODUCTION) {
        // Logic to send the "Hello" message
        txIntroduction(introMsgPtr);
        
        // In RTOS, we don't need complex timer interrupts for the next part;
        // we just increment our pointer and the next loop (1s later) sends the next part.
        if (introMsgPtr < introMsgCnt) {
            introMsgPtr++;
        } else {
            FLAG_SEND_INTRODUCTION = false;
        }
    }
}

/**
 * @brief Task responsible for pulling messages from the Queue and 
 * pushing them to the FDCAN hardware.
 * Uses osMessageGet and HAL_FDCAN_AddMessageToTxQueue
 */
void StartCanTxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *msg;
    FDCAN_TxHeaderTypeDef TxHeader;

    /* Pre-configure static header fields for efficiency */
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // Or FDCAN_FD_CAN if using FD
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    for(;;) {
        // Wait forever for a message to appear in the queue
        event = osMessageGet(canTxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            msg = (CAN_Msg_t*)event.value.p; // Get pointer from queue

            // Update dynamic header fields
            TxHeader.Identifier = msg->Identifier;
            TxHeader.DataLength = FDCAN_DLC_BYTES_8; // Usually fixed at 8 in your project

            // Check if there is space in the Tx FIFO/Queue
            if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, msg->Data) != HAL_OK) {
                    SecureDebug("CAN TX Error\r\n");
                }
            } else {
                // Bus might be congested or in error state
                SecureDebug("CAN TX Buffer Full\r\n");
            }
            
            /* If using Memory Pools, you MUST free the pointer here */
            // osPoolFree(canMsgPoolHandle, msg);
        }
    }
}

/* Memory Pools: https://www.freertos.org/a00111.html */
/* Memory pool examples from Gemini */

osPoolId canMsgPoolHandle;
osPoolDef(canMsgPool, 16, CAN_Msg_t); // Define pool for 16 messages

// Inside MX_FREERTOS_Init
canMsgPoolHandle = osPoolCreate(osPool(canMsgPool));

void StartDefaultTask(void const * argument) {
    for(;;) {
        // 1. Allocate a block from the pool
        CAN_Msg_t *newMsg = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle);
        
        if (newMsg != NULL) {
            // 2. Fill the data
            newMsg->Identifier = 0x123;
            *(uint32_t*)&newMsg->Data[0] = nodeInfo.nodeID; 

            // 3. Put the POINTER into the queue
            if (osMessagePut(canTxQueueHandle, (uint32_t)newMsg, 0) != osOK) {
                // If queue full, we MUST free it or we leak memory
                osPoolFree(canMsgPoolHandle, newMsg);
            }
        }
        osDelay(100);
    }
}

/* Define the Pool: 16 blocks of type CAN_Msg_t */
osPoolId canMsgPoolHandle;
osPoolDef(canMsgPool, 16, CAN_Msg_t); 

/* Define the Queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canTxQueueHandle;
osMessageQDef(canTxQueue, 16, uint32_t);

void MX_FREERTOS_Init(void) {
    /* Create the Memory Pool */
    canMsgPoolHandle = osPoolCreate(osPool(canMsgPool));

    /* Create the Message Queue */
    canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);
}
// Consumer Task
void StartCanTxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pMsg;
    FDCAN_TxHeaderTypeDef txHeader;

    /* Initialize static header fields */
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    for(;;) {
        /* 1. Wait for a pointer to arrive in the queue */
        event = osMessageGet(canTxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            /* 2. Retrieve the pointer to the CAN_Msg_t in the pool */
            pMsg = (CAN_Msg_t*)event.value.p;

            if (pMsg != NULL) {
                txHeader.Identifier = pMsg->Identifier;

                /* 3. Check for hardware space and transmit */
                if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, pMsg->Data) != HAL_OK) {
                        SecureDebug("CAN HW Error\r\n");
                    }
                } else {
                    SecureDebug("CAN Mailbox Full\r\n");
                }

                /* 4. IMPORTANT: Free the memory back to the pool so it can be reused */
                osPoolFree(canMsgPoolHandle, pMsg);
            }
        }
    }
}

// Producer Task
void SendTelemetry(void) {
    /* 1. Allocate a 'blank' message from the pool (non-blocking) */
    CAN_Msg_t *pNew = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle);

    if (pNew != NULL) {
        /* 2. Fill it with data */
        pNew->Identifier = 0x123;
        *(uint32_t*)&pNew->Data[0] = nodeInfo.nodeID; 

        /* 3. Push the POINTER to the queue */
        if (osMessagePut(canTxQueueHandle, (uint32_t)pNew, 0) != osOK) {
            /* If the queue is full, we must manually free the pool block 
               otherwise we create a memory leak! */
            osPoolFree(canMsgPoolHandle, pNew);
        }
    }
}

// RX interrupt handler FDCAN callback
/* Inside app_freertos.c or main.c */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];

        /* 1. Pull message from hardware FIFO immediately */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            
            /* 2. Allocate from Pool (Non-blocking because we are in an ISR!) */
            CAN_Msg_t *pNew = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle);
            
            if (pNew != NULL) {
                pNew->Identifier = rxHeader.Identifier;
                memcpy(pNew->Data, rxData, 8);

                /* 3. Send POINTER to the RX Queue */
                if (osMessagePut(canRxQueueHandle, (uint32_t)pNew, 0) != osOK) {
                    osPoolFree(canMsgPoolHandle, pNew); // Drop if queue full
                }
            }
        }
    }
}

/* The RX Logic Task (The "Brain")
* This task replaces the while(can1.read()) loop from your Arduino code. 
* It sits blocked on osMessageGet, consuming zero CPU until the ISR pushes a pointer into the queue.
*/
void StartCanRxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pRx;

    for(;;) {
        /* 1. Wait for the ISR to feed us a message pointer */
        event = osMessageGet(canRxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            pRx = (CAN_Msg_t*)event.value.p;

            /* 2. Process logic (Similar to your handle_rx_message in main.cpp) */
            uint32_t remoteID = *(uint32_t*)&pRx->Data[0];
            bool isForMe = (remoteID == nodeInfo.nodeID);

            switch (pRx->Identifier) {
                case MSG_NORM_OPER:
                    FLAG_BEGIN_NORMAL_OPER = true;
                    break;

                case MSG_REQ_INTRO:
                    introMsgPtr = 0;
                    FLAG_SEND_INTRODUCTION = true;
                    break;

                case SW_SET_ON:
                    if (isForMe) {
                        uint8_t subIdx = pRx->Data[4];
                        // Update our local struct state
                        nodeInfo.subModules[subIdx].data.u8Value = 1;
                        // Physical IO
                        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
                    }
                    break;
            }

            /* 3. ALWAYS free the pool memory after processing */
            osPoolFree(canMsgPoolHandle, pRx);
        }
    }
}

// Install the interrupt, in fdcan.c
/* Activate notifications for new messages in FIFO 0 */
/* Crucial Configuration Step
* For the callback to actually trigger, you must enable the interrupts in your FDCAN initialization 
* (usually in main.c or fdcan.c). Without this line, the hardware will receive messages but never tell the CPU.
*
* Add this after HAL_FDCAN_Start():
*/
HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


// saving this for future reference
// it formats the adc values and prints them nicely
void StartLCDTask(void const * argument)
{
  /* USER CODE BEGIN StartLCDTask */
  static char msg[64] __attribute__((aligned(8)));



  // osDelay(1000); 
  /* Infinite loop */
  for(;;)
  {
    // 3. No Polling! The data is already in the buffer.
    uint32_t rawAn1 = adc_buffer[0];
    uint32_t rawTemp = adc_buffer[1];

    /* --- Corrected Calculation --- */
    // 1. Adjust rawTemp to a 3.0V reference (assuming your VCC is 3.3V)
    // 918 * 3300 / 3000 = ~1010
    int32_t rawTemp_30V = (int32_t)rawTemp * 3300 / 3000;

    // 2. Calculate the difference from the 30C calibration point
    int32_t delta_ADC = rawTemp_30V - (int32_t)(*TS_CAL1);

    // 3. Scale and divide
    // STM32G0 TS_CAL2 is at 130C. Range is (130 - 30) = 100. Multiply by 10 for decimal (1000).
    int32_t temp_scaled = (delta_ADC * 1000) / (int32_t)(*TS_CAL2 - *TS_CAL1) + 300;

    int t_int = temp_scaled / 10;
    int t_dec = abs(temp_scaled % 10);

    /* --- Printing --- */
    snprintf(msg, sizeof(msg), "\r\nA1:%u T:%d.%dC\r\n", 
              (unsigned int)rawAn1, t_int, t_dec);

    SecureDebug(msg); // Print to UART2

    osDelay(1000); // The DMA updates much faster than this loop
  }
 