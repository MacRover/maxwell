
#include <RadioLib.h>



// SX1262 object (NSS, DIO1, NRESET, BUSY)
SX1262 radio = new Module(10, 3, 40, 39);


volatile bool receivedFlag = false;


int16_t rxState = RADIOLIB_ERR_NONE;


String rxData;


enum RxState {
  RX_INIT,         
  RX_START_LISTEN,
  RX_WAIT_FLAG,   
  RX_READ_DATA,     
};

RxState rxStateMachine = RX_INIT;


#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void onPacketReceived() {
  receivedFlag = true;
}


void setup() {
  Serial.begin(9600);
 
}


void loop() {
  switch (rxStateMachine) {

    
    case RX_INIT: {
      Serial.print(F("[RX] Initializing SX1262 ... "));
      int16_t state = radio.begin(); 
      if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("FAILED, code "));
        Serial.println(state);
        while (true) {
          delay(500);
          Serial.println(F("[RX] Radio init failed; halting."));
        }
      }
      Serial.println(F("OK"));

      
      radio.setPacketReceivedAction(onPacketReceived);

  
      rxStateMachine = RX_START_LISTEN;
      break;
    }

   
    case RX_START_LISTEN: {
      Serial.println(F("[RX] Starting to listen for packets ..."));
      rxState = radio.startReceive();
      if (rxState != RADIOLIB_ERR_NONE) {
        Serial.print(F("[RX] startReceive() failed, code "));
        Serial.println(rxState);
       
        delay(500);
        rxStateMachine = RX_START_LISTEN;
      } else {
       
        rxStateMachine = RX_WAIT_FLAG;
      }
      break;
    }

    
    case RX_WAIT_FLAG: {
      if (receivedFlag) {
        rxStateMachine = RX_READ_DATA;
      }
    
      break;
    }

   
    case RX_READ_DATA: {
      receivedFlag = false;
      int16_t state = radio.readData(rxData);
      if (state == RADIOLIB_ERR_NONE) {
    
        Serial.println(F("[RX] Packet received!"));
        Serial.print(F("  >> \""));
        Serial.print(rxData);
        Serial.println(F("\""));
    
        Serial.print(F("  RSSI: "));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
        Serial.print(F("  SNR:  "));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));
        Serial.print(F("  FreqErr: "));
        Serial.print(radio.getFrequencyError());
        Serial.println(F(" Hz"));
      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[RX] CRC error; packet malformed."));
      } else {
        Serial.print(F("[RX] readData() failed, code "));
        Serial.println(state);
      }
      rxStateMachine = RX_START_LISTEN;
      break;
    }
  }
}