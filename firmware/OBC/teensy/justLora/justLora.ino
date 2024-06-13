#include "lora.h"

#define USING_LORA

Lora lora;

// 124884 --> OBC
// 124873 --> Breakout

void setup()
{
    Serial.begin(115200);
    lora.InitializeLora();
    lora.InitInterruptHandler();
}


void loop()
{
#ifdef USING_LORA
  if (transmittedFlag) {
    transmittedFlag = false;

    Serial.println("In ISR! State: " + lora.GetState());

    lora.EndInterruptTransmit();
  }
  if (receivedFlag) {
    receivedFlag = false;
  }
  if (operationDone) {
    operationDone = false;

  }
  if (scanDone) {
    scanDone = false;

  }
#endif

  if (lora.GetMode() != LORA_MODE::TRANSMIT) {
    lora.InterruptTransmit("Hello Vaibhav!");
  }

}
