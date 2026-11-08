#include "cooling.h"
#include "TSB.h"
#include "fans.h"

Adafruit_MCP9601 MCP;
Fan fan1, fan2, fan3;

TSB_STATES state_TSB = TSB_INIT;
fan_states state_fans = FANS_INIT;

unsigned long prev_time_fan = 0;
unsigned long prev_time_tsb = 0;

bool obc_setup_tsb() 
{
#ifdef USING_TSB
    if (!MCP.begin(MCP9601_ADDR, &Wire1)) { return false; }
    MCP.setADCresolution(MCP9600_ADCRESOLUTION_18);
    MCP.setThermocoupleType(MCP9600_TYPE_K);
    tsb_init();  
#endif
    return true;
}

bool obc_setup_fans()
{
#ifdef USING_FANS
    initializeFan(&fan1, 5);
    initializeFan(&fan2, 6);
    // initializeFan(&fan3, 7);

    enableFanControl(&fan1);
    enableFanControl(&fan2);
    // enableFanControl(&fan3);
#endif
    return true;
}

void FANS_SM() {
    switch (state_fans) {
        case FANS_INIT:
            if (obc_setup_fans()) {
                state_fans = FANS_OK;
            } 
            else {
                state_fans = FANS_ERROR;
            }
            break;

        case FANS_OK:
            // FUTURE INTEGRATION: This is exactly where you will eventually 
            // read tsb1.ambient_temp and dynamically change setFanRPM!
            setFanRPM(&fan1, MIN_RPM);
            setFanRPM(&fan2, MIN_RPM);
            break;

        case FANS_ERROR:
            if (millis() - prev_time_fan > 5000){
                prev_time_fan = millis();

                if (obc_setup_fans()){
                    state_fans = FANS_OK;
                }
                else{
                    state_fans = FANS_ERROR;
                }
            }
            break;

        default:
            state_fans = FANS_INIT;
            break;
    }
}

void TSB_SM(){
    switch(state_TSB){
        case TSB_INIT:
            if (obc_setup_tsb()){
                state_TSB = TSB_OK;
            } 
            else {
                state_TSB = TSB_ERROR;
            }
            break;

        case TSB_OK:
            tsb_update(&MCP); 
            break; 

        case TSB_ERROR:
            if (millis() - prev_time_tsb > 5000) {
                prev_time_tsb = millis();
                if (obc_setup_tsb()){
                    state_TSB = TSB_OK;
                }
                else {
                    state_TSB = TSB_ERROR;
                }
            }
            break;

        default:
            state_TSB = TSB_INIT;
            break;
    }
}

void update_cooling() {
#ifdef USING_TSB
    TSB_SM();
#endif 
#ifdef USING_FANS
    FANS_SM();
#endif
}
