#ifndef _LORA_ISR_H
#define _LORA_ISR_H

// Reference the variables below in main loop when you want to execute a function.
// Ensure you :
// 1. Call InitInterruptHandler() first
// 2. Call interrupt-based methods rather than blocking-based methods
// 3. clear respective flag in main loop (set to 0)
// 4. Do relevant actions
// 5. Execute any clean up functions (EndInterruptTransmit() etc.)

// flag to indicate that a packet was received
volatile bool receivedFlag = false;
// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;
// flag to indicate that a packet was sent or received
volatile bool scanDone = false;
// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// ============================DO NOT EDIT ANY OF THE BELOW============================

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagReceive(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}
// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagTransmit(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}
// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagScan(void) {
  // we sent or received a packet, set the flag
  scanDone = true;
}
// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagOperation(void) {
  // we sent or received a packet, set the flag
  operationDone = true;
}

#endif
