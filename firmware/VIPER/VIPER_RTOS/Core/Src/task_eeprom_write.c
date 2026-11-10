#include "task_eeprom_write.h"

/*
 * TODO: For now, VIPER_PARAMS_TypeDef carries settings, but it will be changed to a fault/state 
        logging as described in the flowcharts.
 */


void vEepromWrite(void *pvParameters){
    
    (void)pvParameters;

    VIPER_PARAMS_TypeDef receivedParams;

    // Counts how many messages couldn't the Tx Queue (useful if inspecting in the debugger) 
    static volatile uint32_t droppedCanReports = 0;

    for (;;){
        // Wait for a save request, with a five-minute timeout.
        if ( xQueueReceive(xEepromWriteQueue, &receivedParams, pdMS_TO_TICKS(300000ul)) != pdPASS){
            // If no request arrived, we save the current parameters instead.
            // [TODO : will add a shared-state protection for this snapshot]
            receivedParams = viper_params;
        }
        
        AT24C04C_StatusTypeDef status = AT24C04C_BUSY;

        
        if( xSemaphoreTake( xI2C1Mutex, pdMS_TO_TICKS(10) ) == pdTRUE )
        {
            //Keep exclusive access to I2C1 throughout the write
            status = AT24C04C_WritePages(&at24c04c_1, (uint8_t*) &receivedParams, sizeof(VIPER_PARAMS_TypeDef), VIPER_PARAMS_EEPROM_PAGE);
            xSemaphoreGive( xI2C1Mutex );
        }
        
        // Pass the write result to the CAN task for transmission
        CanTxMessage eepromCanTxMessage= {0};
        eepromCanTxMessage.type = CAN_EVENT_EEPROM_STATUS;
        eepromCanTxMessage.data.eepromStatus = status;
        
        if (xQueueSend(xCanTxQueue, &eepromCanTxMessage, pdMS_TO_TICKS(10U)) != pdPASS){
            // if the queue is full, count the lost report and continue.
            ++droppedCanReports;
        } 
        // Signals that the task has ran. ! Doesn't mean a sucess. this bit merely indicates the task's liveness.
        xEventGroupSetBits(xWatchdogEventGroup, WDG_EEPROM);
    }

}

