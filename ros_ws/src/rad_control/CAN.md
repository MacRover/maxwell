# RAD CAN Communication

## CAN Frame Format
| **B28 - B16** | **B15 - B8** | **B7 - B0** |
|-----------|----------|---------|
| Unused | Command ID | RAD ID |

## Control Commands (Ingoing)
| **Command Name** | **Command ID** | **Description**
|--------------|------------|-----------|
|SET TARGET ANGLE | 0x4 | Set the target angle, between 0 and 360 degrees (send as double)|
|SET STEPPER SPEED | 0x3 | Set Stepper Speed |
|SET P VALUE | 0x5 | Set P value (sent as a single - 4 bytes)|
|SET I VALUE | 0x7 | Set I value (sent as a single - 4 bytes)|
|SET D VALUE | 0x8 | Set D value (sent as a single - 4 bytes)|
|~~SET DRVCTRL REGISTER~~| ~~0x11~~ | ~~Set register value, sent over 3 bytes~~|
|~~SET CHOPCONF REGISTER~~ | ~~0x13~~ | ~~Set register value, sent over 3 bytes~~|
|~~SET SMARTEN REGISTER~~ | ~~0x15~~ | ~~Set register value, sent over 3 bytes~~|
|~~SET SGSCONF REGISTER~~ | ~~0x17~~ | ~~Set register value, sent over 3 bytes~~|
|~~SET DRVCONF REGISTER~~ | ~~0x19~~ | ~~Set register value, sent over 3 bytes~~|
|~~SET RAD TYPE~~ | ~~0x21~~ | ~~Configure RAD to either RAD_ARM or RAD_DRIVETRAIN, to configure default gear ratios and range of motions (value of TBD RAD ENUM)~~|
|~~CAPTURE HOME POSITION~~ | ~~0x23~~ | ~~Locally store current position as home position (value ignored)~~|
|~~SET ADC1 LOW THRESH~~ | ~~0x25~~ | ~~Used for analog watchdog thresholds to configure interrupts. Between 0 and 2048 as a uint16_t (capturing 0 to 1.16v)~~|
|~~SET ADC1 HIGH THRESH~~ | ~~0x27~~ | " | 
|~~SET ADC2 LOW THRESH~~ | ~~0x29~~ | " | 
|~~SET ADC2 HIGH THRESH~~ | ~~0x31~~ | " |
|SET RAD FLAGS | 0x33 | 4 Bytes to set local flags for RAD|
|SET WATCHDOG INTERVAL | 0x35 | In ms (uint16_t)|
|SAVE TO EEPROM | 0x37 | Data ignored 

## Status Commands (Outgoing)
| **Command Name** | **Command ID** | **Description**
|--------------|------------|-----------|
|STATUS 1 | 0x9 |Limit Switch and Upper Bound State, Current Angle Reading (0 to 360 degrees) |
|STATUS 2 | 0x14 | P-Value, I-Value|
|STATUS 3| 0x15 | D-Value, Motor Speed|
|~~GET DRVCTRL REGISTER~~ | ~~0x12~~ | ~~Get register value, sent over 3 bytes~~|
|~~GET CHOPCONF REGISTER~~ | ~~0x14~~ | ~~Get register value, sent over 3 bytes~~|
|~~GET SMARTEN REGISTER~~ | ~~0x16~~ | ~~Get register value, sent over 3 bytes~~|
|~~GET SGSCONF REGISTER~~ | ~~0x18~~ | ~~Get register value, sent over 3 bytes~~|
|~~GET DRVCONF REGISTER~~ | ~~0x20~~ | ~~Get register value, sent over 3 bytes~~|
|~~GET RAD TYPE~~ | ~~0x22~~ | ~~Configure RAD to either RAD_ARM or RAD_DRIVETRAIN, to configure default gear ratios and range of motions (value of TBD RAD ENUM)~~|
|~~RESET HOME POSITION~~ | ~~0x24~~ | ~~Erase home position, reset to 0, 0 (value is ignored)~~|
|~~GET ADC1 LOW THRESH~~ | ~~0x26~~ | ~~Get analog watchdog thresholds to configure interrupts. Between 0 and 2048 as a uint16_t (capturing 0 to 1.16v)~~ |
|~~GET ADC1 HIGH THRESH~~ | ~~0x28~~ | " | 
|~~GET ADC2 LOW THRESH~~ | ~~0x30~~ | " | 
|~~GET ADC2 HIGH THRESH~~ | ~~0x32~~ | " |
|~~GET RAD FLAGS~~ | ~~0x34~~ | ~~4 Bytes to set local flags for RAD~~|
|~~GET WATCHDOG INTERVAL~~ | ~~0x36~~ | ~~In ms (uint16_t)~~|
