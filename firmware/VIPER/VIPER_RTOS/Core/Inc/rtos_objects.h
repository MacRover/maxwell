#ifndef VIPER_RTOS_RTOS_OBJECTS_H
#define VIPER_RTOS_RTOS_OBJECTS_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "at24c04c.h"
#include "viper_app.h"

extern QueueHandle_t xCanRxQueue;
extern QueueHandle_t xCanTxQueue;
extern QueueHandle_t xEepromWriteQueue;
extern SemaphoreHandle_t xI2C2Mutex;
extern SemaphoreHandle_t xI2C1Mutex;
extern EventGroupHandle_t xWatchdogEventGroup;


typedef enum {
    CAN_EVENT_EEPROM_STATUS,
    CAN_EVENT_CARD_FAULT,
    CAN_EVENT_CARD_TELEMETRY
} CanEventType;

typedef struct {
    CanEventType type;

    union {
        AT24C04C_StatusTypeDef eepromStatus;

        struct {
            uint8_t cardId;
            uint16_t faultCode;
        } cardFault;

        struct {
            VIPER_CARD_ID_TypeDef cardId;
            VIPER_CARD_STATE_TypeDef snapshot;
        } cardTelemetry;
    } data;
} CanTxMessage;
#define CAN_TX_QUEUE_LENGTH 8


 typedef struct {
    uint32_t canid;
    bool is_extended;
    bool is_remote;
    uint8_t dlc;
    uint8_t data[8]; 
 }  CanRxMessage; 

 #define CAN_RX_QUEUE_LENGTH 8
#define EEPROM_WRITE_QUEUE_LENGTH 4

void RTOS_Objects_Init(void);

#endif /* VIPER_RTOS_RTOS_OBJECTS_H */
