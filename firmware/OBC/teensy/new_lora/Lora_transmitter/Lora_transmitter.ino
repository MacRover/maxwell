#include <RadioLib.h>
SX1262 radio = new Module(10, 3, 40, 39); // CS, DIO1, NRST, BUSY

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.startReceive();  
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');

    radio.transmit(line);
    radio.startReceive();  
  }

}