#include <RadioLib.h>
volatile bool receivedFlag = false;
SX1262 radio = new Module(10, 3, 40, 39);

void setFlag() { receivedFlag = true; }

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setPacketReceivedAction(setFlag);
  radio.startReceive();
}

void loop() {
  if (receivedFlag) {
    receivedFlag = false;
    String msg;
    if (radio.readData(msg) == RADIOLIB_ERR_NONE) {
      Serial.println(msg); 
    }
    radio.startReceive();
  }
}