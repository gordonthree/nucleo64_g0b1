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
osPoolDef(canMsgPool, CAN_MSG_POOL_LEN, CAN_Msg_t); 

/* Define the TX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canTxQueueHandle;
osMessageQDef(canTxQueue, CAN_TX_QUEUE_LEN, uint32_t);

/* Define the RX pointer queue: 16 slots, each holding a POINTER (uint32_t) */
osMessageQId canRxQueueHandle;
osMessageQDef(canRxQueue, CAN_RX_QUEUE_LEN, uint32_t);

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

static void Handle_ReqNodeIntro(CAN_Msg_t *pMsg);
static void Handle_AckIntro(CAN_Msg_t *pMsg);
static void Handle_DataEpoch(CAN_Msg_t *pMsg);
static void Handle_SensorGroup(CAN_Msg_t *pMsg);
static void Handle_SystemGroup(CAN_Msg_t *pMsg);
static void Handle_SwitchGroup(CAN_Msg_t *pMsg);
static void Handle_LedStripGroup(CAN_Msg_t *pMsg);
static void Handle_DisplayGroup(CAN_Msg_t *pMsg);


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
  osThreadDef(canRxTask, StartCanRxTask, osPriorityHigh, 0, 512);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  /* definition and creation of canTxTask */
  osThreadDef(canTxTask, StartCanTxTask, osPriorityNormal, 0, 512);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);
}

static void SecureDebug(const char* buffer) {
    // Note: use the handle directly or your getter
    if (osMutexWait(uartMutexHandle, 100) == osOK) { /* Wait 100ms for the mutex */
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        osMutexRelease(uartMutexHandle);
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
    static char msg[DEBUG_MSG_SIZE] __attribute__((aligned(8))); /* Buffer to hold message strings */
    uint32_t tickCounter = 0;
    introMsgPtr = 0; /* Reset the pointer */
 
    /* Start the CAN tasks */
    if (canRxTaskHandle != NULL) xTaskNotifyGive(canRxTaskHandle);
    if (canTxTaskHandle != NULL) xTaskNotifyGive(canTxTaskHandle);

    /* Initialize nodeInfo */
    nodeInfo.nodeID = board_crc; 
    nodeInfo.nodeTypeMsg = BOX_MULTI_IO_ID;
    nodeInfo.subModCnt = 2;

    nodeInfo.subModules[0].modType = INPUT_ANALOG_KNOB_ID;
    nodeInfo.subModules[0].dataMsgId = DATA_ANALOG_KNOB_MV_ID;
    nodeInfo.subModules[0].sendFeatureMask = false;

    nodeInfo.subModules[1].modType = NODE_CPU_TEMP_ID;
    nodeInfo.subModules[1].dataMsgId = DATA_NODE_CPU_TEMP_ID;
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

            /* check to see if we need to print a timestamp */
            /* flag is set once when RTC is synced by master */
            if (FLAG_PRINT_TIMESTAMP) {
                FLAG_PRINT_TIMESTAMP = false; /* Reset flag */
                time_t rawtime = (time_t)RTC_Get_Timestamp(&hrtc); /* Read time from RTC */
                struct tm *timeinfo;
                timeinfo = localtime(&rawtime);

                char dbg[128];
                snprintf(dbg, sizeof(dbg), "RTC Sync: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
                        timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                        timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                
                SecureDebug(dbg);
            
            }
        }
    
        /* Increment global tick and reset at a high number to prevent overflow */
        tickCounter++;
        if (tickCounter >= 1000) tickCounter = 0;
    
        /* Base delay of 100ms */
        osDelay(100);
    }
  /* END StartDefaultTask */
}

static void Handle_DataEpoch(CAN_Msg_t *pMsg) {
    uint32_t rxTimeRev;
    
    /* Cast the address of the payload to a byte pointer to satisfy memcpy */
    // memcpy(&rxTimeRev, (uint8_t*)&pMsg->payload, 4);
    memcpy(&rxTimeRev, ((uint8_t*)&pMsg->payload) + 4, 4);    
    uint32_t unixTimestamp = __REV(rxTimeRev);

    /* Logic to set RTC goes here */
    RTC_Set_Timestamp(&hrtc, unixTimestamp); /**< Set RTC hardware using provided timestamp */
    /* Set a flag to print as human readable string */
    FLAG_PRINT_TIMESTAMP = true;
}


static void Handle_ReqNodeIntro(CAN_Msg_t *pMsg) {
    FLAG_BEGIN_NORMAL_OPER = false;
    FLAG_SEND_INTRODUCTION = true;
    introMsgPtr = 0;
    SecureDebug("CAN RX: Master Requested Intro\r\n");
}

static void Handle_AckIntro(CAN_Msg_t *pMsg) {
    if (FLAG_SEND_INTRODUCTION) {
        introMsgPtr++;
        if (introMsgPtr > nodeInfo.subModCnt) {
            FLAG_SEND_INTRODUCTION = false;
            FLAG_BEGIN_NORMAL_OPER = true;
            SecureDebug("CAN RX: Intro Sequence Complete\r\n");
        }
    }
}

/* --- Group 1: Switches (0x112 - 0x11B) --- */
static void Handle_SwitchGroup(CAN_Msg_t *pMsg) {
    /* You can extract common data here, like payload bytes */
    switch (pMsg->canID) {
        case SW_SET_MODE_ID:       SecureDebug("SW: Set Mode\r\n"); break;
        case SW_SET_OFF_ID:        SecureDebug("SW: Set OFF\r\n");  break;
        case SW_SET_ON_ID:         SecureDebug("SW: Set ON\r\n");   break;
        case SW_MOM_PRESS_ID:      SecureDebug("SW: Mom Press\r\n"); break;
        case SW_SET_PWM_DUTY_ID:   SecureDebug("SW: PWM Duty\r\n"); break;
        default: break;
    }
}

/* --- Group 2: Display & OLED (0x200 - 0x20E) --- */
static void Handle_DisplayGroup(CAN_Msg_t *pMsg) {
    switch (pMsg->canID) {
        case SET_DISPLAY_OFF_ID:   SecureDebug("DISP: OFF\r\n");    break;
        case SET_DISPLAY_ON_ID:    SecureDebug("DISP: ON\r\n");     break;
        case SET_DISPLAY_CLEAR_ID: SecureDebug("DISP: Clear\r\n");  break;
        case SET_OLED_FIELD_COLOR_ID: SecureDebug("OLED: Color\r\n"); break;
        case DISPLAY_DATA_MSG_ID:  SecureDebug("DISP: Data RX\r\n"); break;
        default: break;
    }
}

/* --- Group 3: LED Strips (0x210 - 0x216) --- */
static void Handle_LedStripGroup(CAN_Msg_t *pMsg) {
    switch (pMsg->canID) {
        case SET_ARGB_STRIP_COLOR_ID:  SecureDebug("LED: ARGB Color\r\n"); break;
        case SET_ADDR_STRIP_EFFECT_ID: SecureDebug("LED: Effect\r\n");     break;
        case SET_LED_STRIP_BRIGHTNESS_ID: SecureDebug("LED: Bright\r\n");  break;
        case SET_LED_STRIP_OFF_ID:     SecureDebug("LED: OFF\r\n");        break;
        default: break;
    }
}

static void Handle_SystemGroup(CAN_Msg_t *pMsg) {
    switch (pMsg->canID) {
        case ACK_INTRO_ID:      Handle_AckIntro(pMsg);      break; /* 0x400 */
        case REQ_NODE_INTRO_ID: Handle_ReqNodeIntro(pMsg);  break; /* 0x401 */
        case DATA_EPOCH_ID:     Handle_DataEpoch(pMsg);     break; /* 0x40C */
        
        case MSG_NORM_OPER_ID:  
            FLAG_BEGIN_NORMAL_OPER = true;
            FLAG_HALT_NORMAL_OPER  = false;
            break; /* 0x402 */

        case MSG_HALT_OPER_ID:
            FLAG_BEGIN_NORMAL_OPER = false;
            FLAG_HALT_NORMAL_OPER  = true;
            break; /* 0x403 */

        case REQ_NODECHECK_ID:   SecureDebug("SYS: Node Check"); /* 0x404 */
            break;
        case REQ_HEALTHCHECK_ID: SecureDebug("SYS: Health Check"); /* 0x405 */
            break;
        case REQ_IFACE_ID:       SecureDebug("SYS: Request Interfaces"); /* 0x406 */
            break;
        case REQ_SWITCHBOX_ID:   SecureDebug("SYS: Request Switch Boxes"); /* 0x407 */
            break;
        case REQ_BUTTONS_ID:     SecureDebug("SYS: Request Buttons"); /* 0x410 */
            break;
        case REQ_OUTPUTS_ID:     SecureDebug("SYS: Request Outputs"); /* 0x411 */
            break;
        case REQ_DISPLAYS_ID:    SecureDebug("SYS: Request Displays"); /* 0x412 */
            break;
        case REQ_TEMP_SENSOR_ID: SecureDebug("SYS: Request Temp Sensor"); /* 0x413 */
            break;    
        case REQ_VOLT_SENSOR_ID: SecureDebug("SYS: Request Volt Sensor"); /* 0x414 */
            break;  
        case REQ_AMP_SENSOR_ID:  SecureDebug("SYS: Request Amp Sensor"); /* 0x415 */
            break;  
        case REQ_CLOSURE_INPUT_ID: SecureDebug("SYS: Request Closure Input"); /* 0x416 */
            break;  
        case REQ_AMBIENT_LIGHT_ID: SecureDebug("SYS: Request Ambient Light"); /* 0x417 */
            break;  
        case REQ_IMU_SENSORS_ID: SecureDebug("SYS: Request IMU Sensors"); /* 0x418 */
            break;
            
        default: break;
        }
        // HAL_NVIC_SystemReset();
}

static void Handle_SensorGroup(CAN_Msg_t *pMsg) {
    switch (pMsg->canID) {
        /* Analog physical interface data */
        case DATA_BUTTON_DOWN_ID:       SecureDebug("DATA: Button down\r\n"); break; /* 0x500 */
        case DATA_BUTTON_UP_ID:         SecureDebug("DATA: Button up\r\n"); break; /* 0x501 */
        case DATA_KEYSWITCH_LOCK_ID:    SecureDebug("DATA: Keyswitch lock\r\n"); break; /* 0x502 */
        case DATA_KEYSWITCH_UNLOCK_ID:  SecureDebug("DATA: Keyswitch unlock\r\n"); break; /* 0x503 */
        case DATA_DIAL_CLOCKWISE_ID:    SecureDebug("DATA: Dial clockwise\r\n"); break; /* 0x504 */
        case DATA_DIAL_COUNTER_CLOCKWISE_ID: SecureDebug("DATA: Dial counter clockwise\r\n"); break; /* 0x505 */
        case DATA_DIAL_CLICK_ID:        SecureDebug("DATA: Dial click\r\n"); break; /* 0x506 */
        case DATA_RFID_READ_ID:         SecureDebug("DATA: RFID read\r\n"); break; /* 0x507 */
        case DATA_CONTACT_CLOSED_ID:    SecureDebug("DATA: Contact closed\r\n"); break; /* 0x508 */
        case DATA_CONTACT_OPENED_ID:    SecureDebug("DATA: Contact opened\r\n"); break; /* 0x509 */ 
        case DATA_ANALOG_KNOB_MV_ID: SecureDebug("DATA: Analog knob mv\r\n"); break; /* 0x518 */
        case DATA_AMBIENT_LIGHT_USE_PRIV_MSG_ID: SecureDebug("DATA: Ambient light USE PRIV MSG\r\n"); break; /* 0x510 */
        case DATA_OUTPUT_SWITCH_MOM_PUSH_ID: SecureDebug("DATA: Output switch mom push\r\n"); break; /* 0x519 */
        case DATA_OUTPUT_SWITCH_STATE_ID: SecureDebug("DATA: Output switch state\r\n"); break; /* 0x51D */


        /* Temperature, voltage and current Data */
        case DATA_INTERNAL_TEMPERATURE_ID: SecureDebug("DATA: Internal temperature\r\n"); break; /* 0x50A */
        case DATA_INTERNAL_PCB_VOLTS_ID: SecureDebug("DATA: Internal pcb volts\r\n"); break; /* 0x50B */
        case DATA_INTERNAL_PCB_CURRENT_ID: SecureDebug("DATA: Internal pcb current\r\n"); break; /* 0x50C */
        case DATA_EXTERNAL_TEMPERATURE_ID: SecureDebug("DATA: External temperature\r\n"); break; /* 0x50D */
        case DATA_EXTERNAL_VOLTS_ID: SecureDebug("DATA: External volts\r\n"); break; /* 0x50E */
        case DATA_EXTERNAL_CURRENT_ID: SecureDebug("DATA: External current\r\n"); break; /* 0x50F */
        case DATA_NODE_CPU_TEMP_ID: SecureDebug("DATA: Node CPU temp\r\n"); break; /* 0x51A */
        case DATA_NODE_PCB_TEMP_ID: SecureDebug("DATA: Node PCB temp\r\n"); break; /* 0x51C */

        
        /* IMU Data */
        case DATA_IMU_X_AXIS_ID: SecureDebug("DATA: IMU X Axis USE PRIV MSG\r\n"); break; /* 0x511 */
        case DATA_IMU_Y_AXIS_ID: SecureDebug("DATA: IMU Y Axis USE PRIV MSG\r\n"); break; /* 0x512 */
        case DATA_IMU_Z_AXIS_ID: SecureDebug("DATA: IMU Z Axis USE PRIV MSG\r\n"); break; /* 0x513 */
        case DATA_IMU_X_GYRO_ID: SecureDebug("DATA: IMU X Gyro USE PRIV MSG\r\n"); break; /* 0x514 */
        case DATA_IMU_Y_GYRO_ID: SecureDebug("DATA: IMU Y Gyro USE PRIV MSG\r\n"); break; /* 0x515 */
        case DATA_IMU_Z_GYRO_ID: SecureDebug("DATA: IMU Z Gyro USE PRIV MSG\r\n"); break; /* 0x516 */
        case DATA_IMU_TEMPERATURE_ID: SecureDebug("DATA: IMU temperature\r\n"); break; /* 0x517 */
        
        /* Other Data */
        case DATA_NODE_LAST_BOOT_TIMESTAMP_ID: SecureDebug("DATA: Node last boot timestamp\r\n"); break; /* 0x51B */

        /* ARGB Data */
        case DATA_ARGB_BUTTON_COLOR_ID: SecureDebug("DATA: ARGB button color\r\n"); break; /* 0x524 */
        case DATA_ARGB_BUTTON_LED_MODE_ID: SecureDebug("DATA: ARGB button led mode\r\n"); break; /* 0x525 */


        default: 
            /* Useful for debugging non-implemented sensors */
            // SecureDebug("DATA: Unknown Sensor ID\r\n"); 
            break;
    }
}
/* Dispatch table configuration */
typedef struct {
    uint16_t msgID;
    CAN_Handler_t handler;
} CAN_Dispatch_t;

/* We define the ranges in the table. 
   The RX task will now look for the first range that fits.
*/
const CAN_Dispatch_t can_dispatch_table[] = {
    /* --- 0x100 Series (Switches) --- */
    { 0x112,             Handle_SwitchGroup   }, /* 0x112 - 0x11B */
    
    /* --- 0x200 Series (Output/UI) --- */
    { 0x200,             Handle_DisplayGroup  }, /* 0x200 - 0x20E */
    { 0x210,             Handle_LedStripGroup }, /* 0x210 - 0x216 */
    
    /* --- 0x400 Series (System/Management) --- */
    { 0x400,             Handle_SystemGroup   }, /* 0x400 - 0x40C */
    
    /* --- 0x500 Series (Sensor/Telemetry) --- */
    { 0x500,             Handle_SensorGroup   }  /* 0x500 - 0x53F */
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
    // CAN_Msg_t *pRx;
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
            CAN_Msg_t *pRx = (CAN_Msg_t*)event.value.p;
            uint32_t msgRemoteId = __REV(*(uint32_t*)&pRx->payload);

            if (msgRemoteId == nodeInfo.nodeID) {
                /* RANGE-BASED DISPATCH LOGIC */
                for (int i = 0; i < DISPATCH_COUNT; i++) {
                    /* If it's an exact match OR if it's the start of a range 
                       (We assume anything from the table ID up to the next 
                       16 IDs belongs to that group)
                    */
                    if (pRx->canID == can_dispatch_table[i].msgID || 
                       (pRx->canID >= can_dispatch_table[i].msgID && pRx->canID < can_dispatch_table[i].msgID + 0x10)) 
                    {
                        can_dispatch_table[i].handler(pRx);
                        break;
                    }
                }
            }
            osPoolFree(canMsgPoolHandle, pRx);
        }
    }         
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
