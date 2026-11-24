// #include <SoftwareSerial.h>

// // Set up a software serial port for the Blue Pill
// // RX = Pin 10 (Connect to Blue Pill TX)
// // TX = Pin 11 (Connect to Blue Pill RX)
// SoftwareSerial bluePillSerial(10, 11);

// const int ledOnboard = 13;   // Replaces Pico's "LED"
// const int ledExternal = 8;   // Replaces Pico's Pin 15

// unsigned long usbRxTimer = 0;
// unsigned long uartRxTimer = 0;
// const unsigned long BLINK_DURATION_MS = 500;

// void setup() {
//   // 1. Setup USB UART (PC)
//   Serial.begin(115200);
  
//   // 2. Setup Software UART (Blue Pill)
//   bluePillSerial.begin(115200);

//   pinMode(ledOnboard, OUTPUT);
//   pinMode(ledExternal, OUTPUT);

//   // Initial blink sequence to show the code is active
//   for (int i = 0; i < 10; i++) {
//     digitalWrite(ledOnboard, !digitalRead(ledOnboard));
//     digitalWrite(ledExternal, !digitalRead(ledExternal));
//     delay(500);
//   }
  
//   // Ensure both are off after the startup loop
//   digitalWrite(ledOnboard, LOW);
//   digitalWrite(ledExternal, LOW);
// }

// void loop() {
//   unsigned long currentTime = millis();

//   // 1. Read from USB (python-can) and pass to UART (Blue Pill)
//   if (Serial.available()) {
//     char c = Serial.read();
//     bluePillSerial.write(c);
    
//     digitalWrite(ledOnboard, HIGH);
//     usbRxTimer = currentTime;
//   }

//   // 2. Read from UART (Blue Pill) and pass to USB (python-can)
//   if (bluePillSerial.available()) {
//     char c = bluePillSerial.read();
//     Serial.write(c);
    
//     digitalWrite(ledExternal, HIGH);
//     uartRxTimer = currentTime;
//   }

//   // Non-blocking LED timeout
//   if (currentTime - usbRxTimer > BLINK_DURATION_MS) {
//     digitalWrite(ledOnboard, LOW);
//   }

//   if (currentTime - uartRxTimer > BLINK_DURATION_MS) {
//     digitalWrite(ledExternal, LOW);
//   }
// }

#include <AltSoftSerial.h>

// AltSoftSerial strictly uses:
// RX = Pin 8 (Connect to Blue Pill TX A9)
// TX = Pin 9 (Connect to Blue Pill RX A10)
AltSoftSerial bluePillSerial;

const int ledOnboard = 13;   
const int ledExternal = 7;   // Moved to 7 to free up Pin 8

unsigned long usbRxTimer = 0;
unsigned long uartRxTimer = 0;
const unsigned long BLINK_DURATION_MS = 500;

void setup() {
  Serial.begin(115200);
  bluePillSerial.begin(115200);

  pinMode(ledOnboard, OUTPUT);
  pinMode(ledExternal, OUTPUT);

  for (int i = 0; i < 10; i++) {
    digitalWrite(ledOnboard, !digitalRead(ledOnboard));
    digitalWrite(ledExternal, !digitalRead(ledExternal));
    delay(500);
  }
  
  digitalWrite(ledOnboard, LOW);
  digitalWrite(ledExternal, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available()) {
    char c = Serial.read();
    bluePillSerial.write(c);
    
    digitalWrite(ledOnboard, HIGH);
    usbRxTimer = currentTime;
  }

  if (bluePillSerial.available()) {
    char c = bluePillSerial.read();
    Serial.write(c);
    
    digitalWrite(ledExternal, HIGH);
    uartRxTimer = currentTime;
  }

  if (currentTime - usbRxTimer > BLINK_DURATION_MS) {
    digitalWrite(ledOnboard, LOW);
  }

  if (currentTime - uartRxTimer > BLINK_DURATION_MS) {
    digitalWrite(ledExternal, LOW);
  }
}