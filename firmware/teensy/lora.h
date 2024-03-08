#include <SX126x.h>
#include <SPI.h>
#include <cstdint>

// see the library repo for info: https://github.com/tinytronix/SX126x/tree/master
// see SX126X datasheet for full custom programming instructions

// TODO:
// - remove testing functions and make general purpose interface

// communicate via SPI
// Interface from schematic, perspective of teensy:

// LoRa_RXEN : I/O 34 output : Active HIGH
// LoRA_TXEN : I/O 35 output : Active HIGH
// LoRA_NRST : I/O 40 output : Active LOW : Pulled-Up to VCC on board
// LoRA_BUSY : I/O 39 input : Status indicator for LoRa
// SPI_MOSI : I/O 11 output : SPI data pin
// SPI_MISO : I/O 12 input : SPI data pin
// SPI_SCK : I/O 13 output : SPI clock
// LoRa_CS : I/O 10 output : SPI chip select pin

// Parameters:
// Modulation BandWidth (BW_L) : value from 0 to 9 (see datasheet for mapped values) : improves data rate
// Spreading Factor (SF) : value from 5 to 12 : improves receiver sensitivity at the cost of air time
// Coding Rate (CR) : value from 1 to 4 : improves error correction
// Low Data Rate Optimization (LDRO) : for high SF or low BW_L : reduces number of bits per symbol, better tracking

// LoRa Modem:
// 2 types of packet / data frame formats : explicit vs implicit
// explicit describes number of bytes, coding rate and CRC bit in a header on the frame

// Data buffer:
// 256 byte RAM data buffer for data transmission / reception
// accessible by user (teensy) in all modes except sleep

#define RF_FREQUENCY                                433000000 // Hz  center frequency
#define TX_OUTPUT_POWER                             22        // dBm tx output power
#define LORA_BANDWIDTH                              4         // bandwidth=125khz  0:250kHZ,1:125kHZ,2:62kHZ,3:20kHZ.... look for radio line 392                                                               
#define LORA_SPREADING_FACTOR                       7         // spreading factor = 11 [SF5..SF12]
#define LORA_CODINGRATE                             1         // coding rate = [4/5]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false     // variable data payload
#define LORA_IQ_INVERSION_ON                        false
#define LORA_PAYLOADLENGTH                          0         // 0: variable receive length 

#define LORA_PIN_CS 10            //Port-Pin Output: SPI select
#define LORA_PIN_NRST 40          //Port-Pin Output: Reset 
#define LORA_PIN_BUSY 39          //Port-Pin Input:  Busy
#define LORA_PIN_DIO1 0           //Port-Pin Input:  Interrupt DIO1 ; DIO0 and DIO1 are not exposed in OBC 

#define LORA_PIN_TXEN 35          // Enable tx
#define LORA_PIN_RXEN 34          // enable rx

uint8_t* pRxData = new uint8_t[128];
int16_t rv = 0;

SX126x lora(LORA_PIN_CS, LORA_PIN_NRST, LORA_PIN_BUSY, LORA_PIN_DIO1);

void initializeLoRa() {
  lora.begin(SX126X_PACKET_TYPE_LORA,   //LoRa or FSK, FSK currently not supported
             433000000,                 //frequency in Hz
             -3);                       //tx power in dBm

  lora.LoRaConfig(LORA_SPREADING_FACTOR, LORA_BANDWIDTH, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_PAYLOADLENGTH, 
                    false,              //crcOn  
                    false);             //invertIrq
  
  pinMode(LORA_PIN_TXEN, OUTPUT);
  pinMode(LORA_PIN_RXEN, OUTPUT);

  digitalWrite(LORA_PIN_TXEN, HIGH); // enable tx and rx
  digitalWrite(LORA_PIN_RXEN, HIGH);
}

void pollLoRa() {
  uint8_t rxLen = lora.Receive(pRxData, 128);
  if ( rxLen > 0 )
  {
    uint8_t rssi, snr;
    Serial.println(pRxData[0], DEC);
    lora.ReceiveStatus(&rssi, &snr);
    Serial.print("snr: ");
    Serial.println(snr, DEC);
    Serial.print("rssi: ");
    Serial.println(rssi, DEC);
  }
}

uint8_t i;

void sendLoRa() {
  lora.Send(&i, 1,  SX126x_TXMODE_SYNC);
  i++;
}
