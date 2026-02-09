/* app_freertos.c */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>   // For sprintf and snprintf
#include <stdlib.h>  // For abs
#include <string.h>  // For memset (if you use it)
#include "canbus_project.h" /* my various CAN functions and structs */

#include "FreeRTOS.h"
#include "main.h"


/* Private variables ---------------------------------------------------------*/
uint8_t introMsgPtr; /**< pointer for intro messages */

extern uint32_t board_crc; 

osMutexId uartMutexHandle; // CMSIS-RTOS handle
osMutexDef(uartMutex);

extern uint16_t adc_buffer[2];           
extern FDCAN_RxHeaderTypeDef rxHeader;   
extern uint8_t rxData[8];                
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern uint32_t RTC_Get_Timestamp(RTC_HandleTypeDef *hrtc);
extern HAL_StatusTypeDef RTC_Set_Timestamp(RTC_HandleTypeDef *hrtc, uint32_t timestamp);


struct canNodeInfo nodeInfo; /**< Store information about this node */

/* Create the handles for RTOS tasks*/
osThreadId defaultTaskHandle;
osThreadId canTxTaskHandle;
osThreadId canRxTaskHandle;

/* Define the message pool: 16 blocks of type CAN_Msg_t */
osPoolId canMsgPoolHandle;
osPoolDef(canMsgPool, 16, CAN_Msg_t); 

/* Define the TX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canTxQueueHandle;
osMessageQDef(canTxQueue, 16, uint32_t);

/* Define the RX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canRxQueueHandle;
osMessageQDef(canRxQueue, 16, uint32_t);

/* Global function prototypes -----------------------------------------------*/

void StartDefaultTask(void const * argument);
void StartCanRxTask(void const * argument);
void StartCanTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private function prototypes -----------------------------------------------*/

static void txIntroduction(void); /**< Function to send data about this node to the can bus */
static void txSensorData(void); /**< Function to send sensor data to the can bus */
static float internalTempFloat(uint32_t adc_val);
static void SecureDebug(const char* buffer);

/* Dispatch table configuration */
typedef void (*CAN_Handler_t)(CAN_Msg_t *pMsg);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  // Calculate Board ID here to ensure it is ready before any task starts.
  // board_crc = NodeID_GetU32();


  /* BEGIN RTOS_MUTEX */
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* Create the Memory Pool */
  canMsgPoolHandle = osPoolCreate(osPool(canMsgPool));

  /* Create the Message Queue */
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);
  canRxQueueHandle = osMessageCreate(osMessageQ(canRxQueue), NULL);
  /* END RTOS_MUTEX */

  /* Create the thread(s) */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of canRxTask */
  osThreadDef(canRxTask, StartCanRxTask, osPriorityHigh, 0, 256);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  /* definition and creation of canTxTask */
  osThreadDef(canTxTask, StartCanTxTask, osPriorityNormal, 0, 256);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);
}

static void SecureDebug(const char* buffer) {
    // Note: use the handle directly or your getter
    if (osMutexWait(uartMutexHandle, osWaitForever) == osOK) {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
    }
}
void Handle_DataEpoch(CAN_Msg_t *pMsg) {
    uint32_t rxTimeRev;
    
    /* Cast the address of the payload to a byte pointer to satisfy memcpy */
    // memcpy(&rxTimeRev, (uint8_t*)&pMsg->payload, 4);
    memcpy(&rxTimeRev, ((uint8_t*)&pMsg->payload) + 4, 4);    
    uint32_t unixTimestamp = __REV(rxTimeRev);

    /* Logic to set RTC goes here */
    RTC_Set_Timestamp(&hrtc, unixTimestamp); /**< Set RTC hardware using provided timestamp */
    /* Convert to human readable string */
    time_t rawtime = (time_t)unixTimestamp;
    struct tm *timeinfo;
    // timeinfo = localtime(&rawtime);

    // char dbg[128];
    // snprintf(dbg, sizeof(dbg), "RTC Sync: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
    //          timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
    //          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    // SecureDebug(dbg);
}


void Handle_ReqNodeIntro(CAN_Msg_t *pMsg) {
    FLAG_BEGIN_NORMAL_OPER = false;
    FLAG_SEND_INTRODUCTION = true;
    introMsgPtr = 0;
    SecureDebug("CAN RX: Master Requested Intro\r\n");
}

void Handle_AckIntro(CAN_Msg_t *pMsg) {
    if (FLAG_SEND_INTRODUCTION) {
        introMsgPtr++;
        if (introMsgPtr > nodeInfo.subModCnt) {
            FLAG_SEND_INTRODUCTION = false;
            FLAG_BEGIN_NORMAL_OPER = true;
            SecureDebug("CAN RX: Intro Sequence Complete\r\n");
        }
    }
}
/**
 * @brief Calculate the internal CPU temperature using factory calibration.
 * @param adc_val The raw 12-bit ADC reading from the temp sensor channel.
 * @return Temperature in degrees Celsius.
 */
static float internalTempFloat(uint32_t adc_val) {
/* Standard Calibration Addresses for STM32G0 */
    #ifndef TS_CAL1_ADDR
    #define TS_CAL1_ADDR ((uint16_t*)((uint32_t)0x1FFF75A8)) /* 30°C cal point */
    #endif

    #ifndef TS_CAL2_ADDR
    #define TS_CAL2_ADDR ((uint16_t*)((uint32_t)0x1FFF75CA)) /* 130°C cal point */
    #endif

    /* 1. Read factory calibration data */
    uint16_t ts_cal1 = *TS_CAL1_ADDR;
    uint16_t ts_cal2 = *TS_CAL2_ADDR;

    /* 2. Voltage Scaling 
       Factory cal was done at 3.0V. We must scale our 3.3V reading 
       to what it would have been at 3.0V. */
    float vdda_actual = 3.3f; 
    float vdda_cal = 3.0f;
    float adc_scaled = (float)adc_val * (vdda_actual / vdda_cal);

    float temperature;
    
    /* 3. Linear interpolation using the scaled value */
    #ifdef BOARD_STM32G0 
    /* G0 calibration points are 30C and 130C */
    temperature = ((130.0f - 30.0f) / (float)(ts_cal2 - ts_cal1)) * (adc_scaled - (float)ts_cal1) + 30.0f;
    #else
    /* Other series typically use 30C and 110C */
    temperature = ((110.0f - 30.0f) / (float)(ts_cal2 - ts_cal1)) * (adc_scaled - (float)ts_cal1) + 30.0f;
    #endif

    return temperature;
}


static void txSensorData(void) {
    /* Loop through sub-modules to send current readings */
    for (int i = 0; i < nodeInfo.subModCnt; i++) {
        CAN_Msg_t *pNew = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle);
        if (pNew == NULL) return;

        pNew->canID = nodeInfo.subModules[i].dataMsgId; /**< Set CAN MSG ID */
        pNew->DLC = 8; /**< Classic CAN always 8 bytes */

        uint8_t *pData = (uint8_t*)&pNew->payload; /**< Pointer to the payload */

        /* Payload Bytes 0-3: The unique Node ID */
        *(uint32_t*)&pData[0] = __REV(nodeInfo.nodeID);

        /* Handle Float vs Integer based on Message ID */
        if (nodeInfo.subModules[i].modType == NODE_CPU_TEMP_ID) {
            float temp = nodeInfo.subModules[i].data.fltValue;
            /* Copy float bits into a uint32 for the byte swapper */
            uint32_t raw_bits;
            memcpy(&raw_bits, &temp, 4);
            *(uint32_t*)&pData[4] = __REV(raw_bits);
        } else {
            /* Standard integer for Knob */
            *(uint32_t*)&pData[4] = __REV(nodeInfo.subModules[i].data.i32Value);
        }

        if (osMessagePut(canTxQueueHandle, (uint32_t)pNew, 0) != osOK) {
            osPoolFree(canMsgPoolHandle, pNew);
        }
    }
}

/* --- Node Introduction Logic --- */
static void txIntroduction(void) {
    if (FLAG_SEND_INTRODUCTION == false) return; /**< If we're not supposed to send an intro, don't */

    /* Cooldown to prevent spamming the bus every 100ms tick */
    static uint32_t lastSendTick = 0;
    uint32_t currentTick = osKernelSysTick();
    
    if (currentTick - lastSendTick < 500) {
        return; /* Wait at least 500ms before re-transmitting the same packet */
    }
    lastSendTick = currentTick;

    static char msg[512] __attribute__((aligned(8)));
    CAN_Msg_t *pNew = (CAN_Msg_t*)osPoolAlloc(canMsgPoolHandle); /* Allocate from Pool (Non-blocking) */
    if (pNew == NULL) {
      SecureDebug(("TX INTRO: Pool Error\r\n"));
      return;
    }
    uint16_t txMsgID = 0;
    uint8_t payload[2] = {0};

    // --- STEP 0: Introduce the Node itself ---
    if (introMsgPtr == 0) {
        txMsgID = nodeInfo.nodeTypeMsg; // Retrieve node type aka can message ID
        if (txMsgID == 0) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            return;
        }
        snprintf(msg, sizeof(msg), "TX INTRO: NODE INTRO Type %03x\r\n", txMsgID);
        SecureDebug(msg);
        // SecureDebug("TX: NODE INTRO Type %03x\r\n", txMsgID);
        
        // Send the feature mask
        payload[0] = nodeInfo.featureMask[0]; 
        payload[1] = nodeInfo.featureMask[1];    
    } 
    // --- STEP 1+: Introduce Sub-Modules ---
    else {
        uint8_t modIdx = (uint8_t)(introMsgPtr - 1); // Sub-modules start at ptr 1
        
        if (modIdx >= nodeInfo.subModCnt) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            return;
        }

        txMsgID = nodeInfo.subModules[modIdx].modType; // Retrieve module type aka can message ID
        
        if (txMsgID == 0) {
            osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
            introMsgPtr++; /* Error condition, skip empty slot and move to next */
            return;
        }
        snprintf(msg, sizeof(msg), "TX INTRO: MOD INTRO Type %03x at Idx %i\r\n", txMsgID, modIdx);
        SecureDebug(msg);
        // SecureDebug("TX: MOD INTRO Type %03x at Idx %i\r\n", txMsgID, modIdx);
        
        if (nodeInfo.subModules[modIdx].sendFeatureMask) {
            // Send the feature mask
            payload[0] = nodeInfo.subModules[modIdx].featureMask[0];
            payload[1] = nodeInfo.subModules[modIdx].featureMask[1];
        }
    }

    /* Assemble the final 8-byte CAN Frame */
    pNew->canID = txMsgID;

    /* Get a pointer to the start of the payload as a byte array */
    uint8_t *pData = (uint8_t*)&pNew->payload; 

    /*  Bytes 0-3: Hardware Node ID (CRC32) */
    // *(uint32_t*)&pData[0] = nodeInfo.nodeID;
    *(uint32_t*)&pData[0] = __REV(nodeInfo.nodeID); /* Endian Swap */

    /* Bytes 4-5: Feature Mask */
    pData[4] = payload[0];
    pData[5] = payload[1];

    /* Bytes 6-7: Padding */
    pData[6] = 0;
    pData[7] = 0;

    pNew->DLC = 8;

    /* Push the POINTER to the queue */
    if (osMessagePut(canTxQueueHandle, (uint32_t)pNew, 0) != osOK) {
        osPoolFree(canMsgPoolHandle, pNew); /* Release unused pool block */
    }
}

/* USER CODE BEGIN RTOS_THREADS */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    static char msg[512] __attribute__((aligned(8))); /* Buffer to hold message strings */
    uint32_t tickCounter = 0;
    introMsgPtr = 0; /* Reset the pointer */
  
    /* Initialize nodeInfo */
    nodeInfo.nodeID = board_crc; 
    nodeInfo.nodeTypeMsg = BOX_MULTI_IO_ID;
    nodeInfo.subModCnt = 2;

    nodeInfo.subModules[0].modType = INPUT_ANALOG_KNOB_ID;
    nodeInfo.subModules[0].dataMsgId = DATA_ANALOG_KNOB_MV_ID;
    // nodeInfo.subModules[0].dataSize = 2; // 2 bytes
    nodeInfo.subModules[0].sendFeatureMask = false;

    nodeInfo.subModules[1].modType = NODE_CPU_TEMP_ID;
    nodeInfo.subModules[1].dataMsgId = DATA_NODE_CPU_TEMP_ID;
    // nodeInfo.subModules[1].dataSize = 2; // 2 bytes
    nodeInfo.subModules[1].sendFeatureMask = false;

    /* Use the last few bits of your unique ID to create a 'random' startup delay */
    uint32_t jitter = (nodeInfo.nodeID & 0x1FF); 
    osDelay((2000 + jitter));

    snprintf(msg, sizeof(msg), 
          "\r\n\r\n"
          "-----------------------\r\n"
          "--- DEVICE IDENTITY ---\r\n"
          "--NODE ID: 0x%08lX--\r\n"
          "-----------------------\r\n\r\n", 
          (uint32_t)nodeInfo.nodeID);
          
    SecureDebug(msg); // Print to UART2


  
    /* 3. Start the CAN tasks */
    if (canRxTaskHandle != NULL) xTaskNotifyGive(canRxTaskHandle);
    if (canTxTaskHandle != NULL) xTaskNotifyGive(canTxTaskHandle);
        
    /* Set flag to send the introduction*/
    FLAG_SEND_INTRODUCTION = true; 

    /* Infinite loop at 100ms resolution */
    for(;;)
    {
        /* --- SECTION A: HIGH PRIORITY NETWORK LOGIC --- */

        /* Update ADC values */
        /* analog knob */
        nodeInfo.subModules[0].data.i32Value = (uint32_t)adc_buffer[0]; 
        /* Update CPU Temp (calculated as float) */
        nodeInfo.subModules[1].data.fltValue = internalTempFloat(adc_buffer[1]);

        if (FLAG_SEND_INTRODUCTION) {
            /* While in intro mode, we attempt to send the current packet.
               The RxTask will increment introMsgPtr when ACKs arrive. */
            txIntroduction();
        } 
        else if (FLAG_BEGIN_NORMAL_OPER) {
            /* Once intro is done, send sensor data every 1 second (10 ticks) */
            if (tickCounter % 10 == 0) {
                txSensorData();
            }
        }         
        else if (FLAG_SET_RTC_TIME) {
             /* Code to set the RTC time goes here */
        
        }

        /* Check for CAN bus errors */
        FDCAN_ProtocolStatusTypeDef protocolStatus;
        
        /* Get the current status from the hardware */
        HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocolStatus);

        /* 0=No Error, 1=Stuff, 2=Form, 3=Ack, 4=Bit1, 5=Bit0, 6=CRC */
        uint8_t lec = protocolStatus.LastErrorCode; 
        if (lec == 3) {
            SecureDebug("CAN: No ACK detected - check wiring!\r\n");
        }

        /* Check specifically for Bus-Off */
        if (protocolStatus.BusOff) {
            SecureDebug("CAN: Bus-Off detected! Attempting recovery...\r\n");
            
            /* 1. Stop the peripheral */
            HAL_FDCAN_Stop(&hfdcan1);
            
            /* 2. Wait a bit for the bus to stabilize */
            osDelay(100); 
            
            /* 3. Restart the peripheral */
            if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
                /* 4. Re-enable notifications */
                HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
                SecureDebug("CAN: Recovery successful.\r\n");
            } else {
                SecureDebug("CAN: Recovery failed.\r\n");
            }
        }
        
        /* --- SECTION B: LOW PRIORITY UI/HEARTBEAT LOGIC --- */
        /* Toggle LED every 10 ticks (1000ms) */
        if (tickCounter % 10 == 0) {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); /**< Toggle LED */
            // snprintf(msg, sizeof(msg), "\r\nCPU Raw: %ld Flt: %.2f°C Int32: %ld\r\n", adc_buffer[1], internalTempFloat(adc_buffer[1]), internalTempInt(adc_buffer[1])); /**< Periodic CPU temp report */
            // SecureDebug(msg); 
        }
    
        /* Increment global tick and reset at a high number to prevent overflow */
        tickCounter++;
        if (tickCounter >= 1000) tickCounter = 0;
    
        /* Base delay of 100ms */
        osDelay(100);
    }
  /* END StartDefaultTask */
}
/* Dispatch table configuration */
typedef struct {
    uint16_t msgID;
    CAN_Handler_t handler;
} CAN_Dispatch_t;

/* The actual table - make it 'const' to save RAM and put it in Flash */
const CAN_Dispatch_t can_dispatch_table[] = {
    { REQ_NODE_INTRO_ID, Handle_ReqNodeIntro },  /* 0x401 */
    { ACK_INTRO_ID,      Handle_AckIntro      }, /* 0x400 */
    { DATA_EPOCH_ID,     Handle_DataEpoch     }, /* 0x40C */
};

#define DISPATCH_COUNT (sizeof(can_dispatch_table) / sizeof(CAN_Dispatch_t))


/**
 * @brief Function implementing the canRxTask thread.
 * @details This function continuously waits for message pointers from the ISR
 *          and checks if the message is intended for this node. It then
 *          performs the respective actions based on the message type.
 * @param argument: Not used
 * @retval None
 */
void StartCanRxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pRx;
    // char dbg[64]; /* Local stack message buffer */
    
    /* 1. Wait for the signal from StartDefaultTask */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

    /* 2. Drain the queue and free memory safely */
    osEvent drainEvent;
    
    /* Use 0 timeout to poll the queue without blocking */
    while ((drainEvent = osMessageGet(canRxQueueHandle, 0)).status == osEventMessage) {
      /* Now pOld is guaranteed to be assigned only when a message exists */
      CAN_Msg_t *pOld = (CAN_Msg_t*)drainEvent.value.p;
      if (pOld != NULL) {
        osPoolFree(canMsgPoolHandle, pOld);
      }
    }

    for(;;) {
        /* 1. Wait for a message pointer from the ISR */
        event = osMessageGet(canRxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            pRx = (CAN_Msg_t*)event.value.p;

            /* Extract the remote ID */
            uint32_t msgRemoteId = __REV(*(uint32_t*)&pRx->payload); /* Reverse endianness */

            /* Check if the message is for our Node ID */
            if (msgRemoteId == nodeInfo.nodeID) {
                bool found = false;
                
                /* Search the table for a matching ID */
                for (int i = 0; i < DISPATCH_COUNT; i++) {
                    if (pRx->canID == can_dispatch_table[i].msgID) {
                        /* Execute the function pointed to in the table */
                        can_dispatch_table[i].handler(pRx);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    /* Optional: Log unhandled IDs */
                    // snprintf(msg, sizeof(msg), "CAN RX: Unhandled ID: 0x%08lx\r\n", pRx->canID);
                    // SecureDebug(msg);
                }
            }

            /* 4. ALWAYS free the pointer back to the pool */
            osPoolFree(canMsgPoolHandle, pRx);
        }
    }         
        // case SW_SET_ON_ID:
        //     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        //     SecureDebug("CAN RX: Remote Load ON\r\n");
        //     break;

        // case SW_SET_OFF_ID:
        //     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
        //     SecureDebug("CAN RX: Remote Load OFF\r\n");
        //     break;
}

/**
 * @brief  CAN Transmitter Task
 * @details  This task waits for pointers to CAN_Msg_t structures to be pushed into the canTxQueueHandle by other tasks.
 *             It then transmits the data over the CAN bus using the HAL_FDCAN library.
 *             The task is responsible for checking for hardware space and releasing memory back to the pool.
 * @param  argument: Unused
 * @retval None
 */
void StartCanTxTask(void const * argument) {
    osEvent event;
    CAN_Msg_t *pMsg;
    FDCAN_TxHeaderTypeDef txHeader;
    
    /* Wait for the signal from StartDefaultTask */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* Initialize static header fields */
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;

    for(;;) {
        /* 1. Wait for a pointer to arrive in the queue */
        event = osMessageGet(canTxQueueHandle, osWaitForever);

        if (event.status == osEventMessage) {
            /* 2. Retrieve the pointer to the CAN_Msg_t in the pool */
            pMsg = (CAN_Msg_t*)event.value.p;

            if (pMsg != NULL) {
                txHeader.Identifier = pMsg->canID;

                /* 3. Check for hardware space and transmit */
                if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
                  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)&pMsg->payload) != HAL_OK) {
                      SecureDebug("CAN HW Error\r\n");
                  }
                } else {
                  int retry = 0;
                  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0 && retry < 5) {
                      osDelay(2); /* Give the hardware 5ms to clear a mailbox */
                      retry++; 
                  }
                  if (retry >= 5) SecureDebug("CAN Mailbox Full\r\n");
                }

                /* 4. IMPORTANT: Free the memory back to the pool so it can be reused */
                osPoolFree(canMsgPoolHandle, pMsg);
            }


        }
    }
}
