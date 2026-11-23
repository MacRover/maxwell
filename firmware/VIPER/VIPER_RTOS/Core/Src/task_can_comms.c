#include "task_can_comms.h"
#include "can.h"
#include "rtos_objects.h"
#include "task.h"

// [TODO remove these global variables]
static VIPER_STATE_TypeDef snapshot;

static VIPER_CARD_STATE_TypeDef *const card_states[] = {
    [VIPER_CARD_0] = &snapshot.CARD_0,
    [VIPER_CARD_1] = &snapshot.CARD_1,
    [VIPER_CARD_2] = &snapshot.CARD_2,
    [VIPER_CARD_3] = &snapshot.CARD_3,
};


static void send_health(bool requested)
{
    // [todo] Use xViperStateMutex once the sensor and fault tasks publish state.
    taskENTER_CRITICAL();
    snapshot = viper_state;
    taskEXIT_CRITICAL();

    MX_CAN_Broadcast_Health_Message(&viper_can, &snapshot);
    if (requested) {
        VIPER_TEST_REPORT(HEALTH_STATUS_PING | 0x80U, 0, snapshot.STATE);
    }
}

static void handle_rx(const CanRxMessage *rx)
{
    const uint32_t id = rx->canid;
    const uint32_t reply_bit = 1UL << CAN_MESSAGE_RESPONSE_OFFSET;
    if (!rx->is_extended || rx->is_remote || rx->dlc > sizeof(rx->data) || id > 0x1FFFFFFFUL || (id & reply_bit) != 0U) {
        // [Maybe TODO : some debugging info if useful]
        return;
    }

    const uint32_t scope = (id >> CAN_MESSAGE_IDENTIFIER_OFFSET) & CAN_MESSAGE_IDENTIFIER_MASK;
    const uint8_t command = (id >> CAN_MESSAGE_COMMAND_OFFSET) & CAN_MESSAGE_COMMAND_MASK;

    if (scope == CAN_MESSAGE_IDENTIFIER_GLOBAL) {
        if (command == HEALTH_STATUS_PING) {
            send_health(true);
        }
        // [todo] When TaskFaultHandler is ready, queue global ENABLE/DISABLE and notify it.
        // E-stop uses the urgent notification path in the RX ISR.
        return;
    }

    if (scope != CAN_MESSAGE_IDENTIFIER_VIPER ||
        ((id >> CAN_MESSAGE_DEVICE_ID_OFFSET) & CAN_MESSAGE_DEVICE_ID_MASK)
            != viper_can.id) {
        return;
    }

    switch (command) {
    case GET_HEALTH_INTERVAL:
    case GET_CARD_INTERVAL: {
        const uint32_t interval = command == GET_HEALTH_INTERVAL ? viper_params.HEALTH_INTERVAL: viper_params.CARD_INTERVAL;
        MX_CAN_Broadcast_Uint32_Data(&viper_can, interval, command, 0);
        VIPER_TEST_REPORT(command, 0, interval);
        break;
    }
    case SAVE_TO_EEPROM:
        if (xQueueSend(xEepromWriteQueue, &viper_params, 0) != pdPASS) {
            VIPER_TEST_REPORT(SAVE_TO_EEPROM, 0, AT24C04C_BUSY);
        }
        break;

    case ENABLE_CARD:
    case ENABLE_ALL_CARDS:
    case DISABLE_CARD:
    case DISABLE_ALL_CARDS:
        // [todo] When TaskFaultHandler is ready, queue the power command and notify it.
        break;

    case GET_CARD_DATA:
    case GET_ALL_CARD_DATA:
    case SET_MUX_VALUE:
    case SET_FREEZE:
    case STOP_FREEZE:
        // [todo] When TaskCardMonitor is ready, read its snapshot for GETs
        // and queue mux/freeze commands to its control interface.
        break;

    case SET_HEALTH_INTERVAL:
    case SET_CARD_INTERVAL:
        // [todo] Wire the setters once CAN and EEPROM share settings protection.
        break;

    default:
        break;
    }
}

void vCanComms(void *argument)
{
    (void)argument;
    CanRxMessage rx;
    CanTxMessage event;
    uint32_t last_health_ms = HAL_GetTick();



    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    for (;;) {
        BaseType_t received = xQueueReceive(xCanRxQueue, &rx, pdMS_TO_TICKS(20U));

        if (ESTOP != 0U) {
            vTaskSuspend(NULL);
            continue;
        }
        if (received == pdPASS) {

            handle_rx(&rx);
        }
        if (xQueueReceive(xCanTxQueue, &event, 0) == pdPASS) {
            switch (event.type) {
            case CAN_EVENT_EEPROM_STATUS:
                viper_state.EEPROM_STATUS = event.data.eepromStatus;
                VIPER_TEST_REPORT(SAVE_TO_EEPROM, 0, event.data.eepromStatus);
                break;
            
            case CAN_EVENT_CARD_TELEMETRY: {
                const VIPER_CARD_ID_TypeDef card_id = event.data.cardTelemetry.cardId;

                *card_states[card_id] = event.data.cardTelemetry.snapshot;
                MX_CAN_Broadcast_Card_Data(&viper_can, &snapshot, card_id);
                break;
            }

            case CAN_EVENT_CARD_FAULT:
                // [todo] Send fault events when TaskFaultHandler publishes them.
                break;
            }
}

        const uint32_t now = HAL_GetTick();
        const uint32_t interval = viper_params.HEALTH_INTERVAL;
        if (interval != 0U && now - last_health_ms >= interval) {
            send_health(false);
            last_health_ms = now;
        }
        // [todo] Send periodic card telemetry when TaskCardMonitor publishes readings

#ifdef VIPER_TEST
        VIPER_TestPoll();
#endif
        MX_CAN_PollToEmptyQueue();
    }
}

