#ifndef _LORA_H
#define _LORA_H

/*
   LORA Firmware using RadioLib SX126x module
   By: Vaibhav Gopalakrishnan, McMaster University

   TODO:
   - Channel + spectral scan for filtering
   - Setting + using node addresses for filtering
   - Testing different LoRa RF settings

    RadioLib currently supports the following settings:
    - pins (SPI slave select, DIO1, DIO2, BUSY pin) (PRESET / CONSTANT)
    - **carrier frequency**
    - **bandwidth**
    - **spreading factor**
    - **coding rate**
    - **sync word**
    - output power during transmission
    - CRC (ALWAYS SET)
    - preamble length
    - TCXO voltage (NOT APPLICABLE)
    - DIO2 RF switch control (NOT APPLICABLE --> NO DIO2 PIN FOR US)

    The settings marked by **[]** have to be the same on both transmitter and receiver,
    (aka. what we care about).

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/


   Usage flow (for interrupt-based):
   1. Create Lora object
   2. Call InitializeLora() and InitInterruptHandler()
   3. For recieve:
    a. Call InterruptReceive() to start listening
    b. Call ReadInterruptPacket() in ISR
    c. Access the last received packet parameters using the class properties named rx_*
   4. For transmit:
    a. Call InterruptTransmit() to transmit packet
    b. Call EndInterruptTransmit() after transmitting in ISR
    c. Access the last transmitted packet parameters using the class properties named tx_*
   5. After each set of actions you can:
    a. Set to standby mode (short-term low-power)
    b. Set to sleep mode (long-term low-power)

   NOTE: Although interrupt-based LoRa interaction is prefered due to less computational time, blocking is much simpler to implement
   and is therefore recommend to get started with testing...
*/

// include the library
#include <RadioLib.h>
#include "lora_isr.h"

// =======================================================================================
// =======================================================================================
// ============================CHANGE THE DEFINE SETTINGS ONLY============================
// =======================================================================================
// =======================================================================================

// The LoRa chip on the OBC has the following connections:
// NSS pin:   10
// DIO1 pin:  3
// NRST pin:  40
// BUSY pin:  39
#define NSS_PIN 10
#define DIO_PIN 3
#define NRST_PIN 40
#define BUSY_PIN 39

// SX1262 has the following defaults:
// - FREQ = 434.0
// - BANDWIDTH = 125.0
// - SPREAD_FACTOR = 9
// - CODING_RATE = 7
// - SYNC_WORD = RADIOLIB_SX126X_SYNC_WORD_PRIVATE
// - POWER = 10
// - PREAMBLE_LENGTH = 8
#define FREQ 434.0
#define BANDWIDTH 125.0
#define SPREAD_FACTOR 9
#define CODING_RATE 7
#define SYNC_WORD RADIOLIB_SX126X_SYNC_WORD_PRIVATE
#define POWER 10
#define PREAMBLE_LENGTH 8

// ====================================================================================
// ====================================================================================
// ============================DO NOT EDIT ANY OF THE BELOW============================
// ====================================================================================
// ====================================================================================

// Mutually exclusive modes
enum LORA_MODE {
  SLEEP,
  STANDBY,
  RECEIVE,
  TRANSMIT,
  SCAN_CHANNEL,
};

class Lora {
  private:
    SX1262 radio = new Module(NSS_PIN, DIO_PIN, NRST_PIN, BUSY_PIN);
    LORA_MODE mode;
    int state;

    void loraPrint(String str) {
      if (verbose) {
        Serial.print(serialID + str);
      }
    }

    void loraPrintln(String str) {
      if (verbose) {
        Serial.println(serialID + str);
        }
    }

    void loraPrintState() {
      if (verbose) {
        Serial.println(serialID + F("Status Code : ") + state);
      }
    }

    void loraHandleError() {
      while (true);
    }

    void loraCheckError(String successMsg, String failMsg) {
      if (state == RADIOLIB_ERR_NONE) {
        loraPrintln(F("Success! ") + successMsg);
      } else {
        loraPrintln(F("Fail! ") + failMsg);
        loraPrintState();
        loraHandleError();
      }
    }

  public:
    // Lora class parameters
    bool verbose;
    String serialID = F("[SX1262] "); // F() macro stores in flash mem

    // Last received packet parameters
    float rx_snr;
    float rx_rssi;
    size_t rx_packetLength;
    float rx_freqError;

    // Last transmitted packet parameters
    float tx_dataRate;

    Lora (bool isVerbose = true) {
      verbose = isVerbose;
    }

    int GetState() {
      return state;
    }

    LORA_MODE GetMode() {
      return mode;
    }

    void InitializeLora() {
      loraPrint(F("Initializing LoRa... "));
      state = radio.begin(FREQ, BANDWIDTH, SPREAD_FACTOR, CODING_RATE, SYNC_WORD, POWER, PREAMBLE_LENGTH);
      loraCheckError("", "");
    }

    void ResetLora() {
      loraPrint(F("Resetting LoRa using NRST Pin ... "));
      state = radio.reset(); // Reset chip using NRST pin
      loraCheckError("", "");
    }

    void InitInterruptHandler() {
      radio.setPacketReceivedAction(setFlagReceive);
      radio.setPacketSentAction(setFlagTransmit);
      //radio.setChannelScanAction(setFlagScan);
      //radio.setDio1Action(setFlagOperation); // DIO1 is interrupt pin, toggled when any of the above interrupts are active
    }

    void ClearInterruptHandler() {
      radio.clearPacketReceivedAction();
      radio.clearPacketSentAction();
      //radio.clearChannelScanAction();
      //radio.clearDio1Action();
    }

    void SetMode(LORA_MODE md) {
      String msg = (md == LORA_MODE::STANDBY) ? F("standby") : ((md == LORA_MODE::SLEEP) ? F("sleep") : F("other"));
      loraPrint(F("Attempting to change power mode to ") + msg + " ... ");
      if (md == LORA_MODE::STANDBY && mode != LORA_MODE::STANDBY) {
        state = radio.standby(); // ON , in standby mode (no active RX/TX)
        loraCheckError("", "");
        mode = LORA_MODE::STANDBY;
      }else if (md == LORA_MODE::SLEEP && mode != LORA_MODE::SLEEP) {
        state = radio.sleep(); // OFF , in sleep mode (low power)
        loraCheckError("", "");
        mode = LORA_MODE::SLEEP;
      } else if (md == mode){
        loraPrintln(F("Already in ") + msg + "!");
      } else {
        loraPrintln(F("Transitioning to ") + msg + F(" mode via SetMode function is not supported."));
      }
    }

    void UpdatePacketLength() {
      rx_packetLength = radio.getPacketLength();
    }

    void UpdateRXPacketParameters() {
      rx_snr = radio.getSNR();
      rx_rssi = radio.getRSSI();
      rx_freqError = radio.getFrequencyError();
    }

    void UpdateTXPacketParameters() {
      tx_dataRate = radio.getDataRate();
    }

    // INTERRUPT RX/TX MODES =================================

    void InterruptReceive() {
      loraPrint(F("Starting to listen ... ")); 
      state = radio.startReceive(); // start listening for LoRa packets
      loraCheckError(F("Starting Recieve!"), F("Failed to start Receive!"));
      mode = LORA_MODE::RECEIVE;
    }

    void ReadInterruptPacket(String str, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      }
      UpdateRXPacketParameters();
      state = radio.readData(str);
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
    }

    void ReadInterruptPacket(byte* byteArr, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      }
      UpdateRXPacketParameters();
      state = radio.readData(byteArr, rx_packetLength); // ensure allocated 256 byte array beforehand for all ranges of data
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
    }

    void ReadInterruptPacket(byte* byteArr, int byteLength, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      }
      UpdateRXPacketParameters();
      state = radio.readData(byteArr, byteLength); // create dynamically allocated byte array beforehand
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
    }

    void InterruptTransmit(String str) {
      loraPrintln(F("Sending packet ... ")); // start transmitting the packet
      state = radio.startTransmit(str); // transmit C-string or Arduino string up to 256 characters long
      mode = LORA_MODE::TRANSMIT;
      // check state error after ISR during main loop

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
      // i.e:  tx_dataRate does not get updated
    }

    void InterruptTransmit(byte* byteArr, int byteLength) {
      loraPrintln(F("Sending packet ... ")); // start transmitting the packet
      state = radio.startTransmit(byteArr, byteLength); // transmit byte array up to 256 bytes long
      mode = LORA_MODE::TRANSMIT;
      // check state error after ISR during main loop

      // NOTE: when using interrupt-driven transmit method,
      //       it is not possible to automatically measure
      //       transmission data rate using getDataRate()
      // i.e:  tx_dataRate does not get updated
    }

    void EndInterruptReceive() {
      SetMode(LORA_MODE::STANDBY);
    }

    void EndInterruptTransmit(bool endOfSequence) {
      if (endOfSequence) {
        state = radio.finishTransmit(); // ALWAYS run after finishing a transmit sequence (can be 1 or multiple transmit function calls in sequence)
        loraCheckError(F("Turned off RF transmit switch!"), F("Failed to clean up transmit sequence!"));
      }
      SetMode(LORA_MODE::STANDBY);
    }

    // BLOCKING TX/RX METHODS =========================================

    void ReadBlockingPacket(String str, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      }
      UpdateRXPacketParameters();
      mode = LORA_MODE::RECEIVE;
      state = radio.receive(str);
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
      mode = LORA_MODE::STANDBY;
    }

    void ReadBlockingPacket(byte* byteArr, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      } 
      UpdateRXPacketParameters();
      mode = LORA_MODE::RECEIVE;
      state = radio.receive(byteArr, rx_packetLength); // ensure allocated 256 byte array beforehand for all ranges of data
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
      mode = LORA_MODE::STANDBY;
    }

    void ReadBlockingPacket(byte* byteArr, int byteLength, bool updatePacketLength = true) {
      if (updatePacketLength) {
        UpdatePacketLength();
      }
      UpdateRXPacketParameters();
      mode = LORA_MODE::RECEIVE;
      state = radio.receive(byteArr, byteLength); // create dynamically allocated byte array beforehand
      loraCheckError(F("Read packet succesfully!"), F("Failed to read packet!"));
      mode = LORA_MODE::STANDBY;
    }

    void BlockingTransmit(String str) {
      loraPrintln(F("Sending packet ... ")); // start transmitting the packet
      mode = LORA_MODE::TRANSMIT;
      state = radio.transmit(str); // transmit C-string or Arduino string up to 256 characters long
      loraCheckError(F("Transmitted packet succesfully!"), F("Failed to transmit packet!"));
      UpdateTXPacketParameters();
      mode = LORA_MODE::STANDBY;
    }

    void BlockingTransmit(byte* byteArr, int byteLength) {
      loraPrintln(F("Sending packet ... ")); // start transmitting the packet
      mode = LORA_MODE::TRANSMIT;
      state = radio.transmit(byteArr, byteLength); // transmit byte array up to 256 bytes long
      loraCheckError(F("Transmitted packet succesfully!"), F("Failed to transmit packet!"));
      UpdateTXPacketParameters();
      mode = LORA_MODE::STANDBY;
    }

    // CHANNEL AND SPECTRAL SCAN METHODS =========================================

    // TODO...

};

#endif