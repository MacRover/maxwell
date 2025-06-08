
#include <RadioLib.h>



// SX1262 object (NSS, DIO1, NRESET, BUSY)
SX1262 radio = new Module(10, 3, 40, 39);


volatile bool received_flag = false;


int16_t reading_state = RADIOLIB_ERR_NONE;


String lora_data;


enum LORA_STATES {
  LORA_INIT,         
  LORA_LISTEN,
  LORA_FLAG,   
  LORA_PRINT,     
} state_lora;




#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void packet_received() {
  received_flag = true;
}


void setup() {
  Serial.begin(9600);
  state_lora = LORA_INIT;
}

void LORA_SM(){
   switch (state_lora) {

    
    case LORA_INIT: {
      Serial.print(F(" Initializing SX1262 ... "));
      int16_t state = radio.begin(); 
      if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("FAILED, code "));
        Serial.println(state);
        while (true) {
          delay(500);
          Serial.println(F(" Radio init failed; halting."));
        }
      }
      Serial.println(F("OK"));

      
      radio.setPacketReceivedAction(packet_received);

  
      state_lora = LORA_LISTEN;
      break;
    }

   
    case LORA_LISTEN: {
      Serial.println(F(" Starting to listen for packets ..."));
      reading_state = radio.startReceive();
      if (reading_state != RADIOLIB_ERR_NONE) {
        Serial.print(F(" startReceive() failed, code "));
        Serial.println(reading_state);
       
        delay(500);
        state_lora = LORA_LISTEN;
      } else {
       
        state_lora = LORA_FLAG;
      }
      break;
    }

    
    case LORA_FLAG: {
      if (received_flag) {
        state_lora = LORA_PRINT;
      }
    
      break;
    }

   
    case LORA_PRINT: {
      received_flag = false;
      int16_t state = radio.readData(lora_data);
      if (state == RADIOLIB_ERR_NONE) {
    
        Serial.println(F(" Packet received!"));
        Serial.print(F("  >> \""));
        Serial.print(lora_data);
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
        Serial.println(F(" CRC error; packet malformed."));
      } else {
        Serial.print(F(" readData() failed, code "));
        Serial.println(state);
      }
      state_lora = LORA_LISTEN;
      break;
    }
  }
}

void loop() {
  LORA_SM();

  delay(1);
} 