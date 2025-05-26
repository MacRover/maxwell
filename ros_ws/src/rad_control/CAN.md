# RAD CAN Communication

## CAN Frame Format
**B28 - B25** |**B24 - B20** |**B19 - B18** | **B17 - B16** | **B15 - B8** | **B7 - B0** |
|------------|-------------|--------------|-----------|----------|---------|
|2| Unused | 0(W)/1(R) | Unused | Command ID | RAD ID |

## Control Commands (Ingoing)
| **Command Name** | **Command ID** | **Description**
|--------------|------------|-----------|
|SET TARGET ANGLE | 1 | Set the target angle, between 0 and 360 degrees (send as double)|
|SET STEPPER SPEED | 3 | Set Stepper Speed |
|SET P VALUE | 5 | Set P value (sent as a single - 4 bytes)|
|SET I VALUE | 7 | Set I value (sent as a single - 4 bytes)|
|SET D VALUE | 9 | Set D value (sent as a single - 4 bytes)|
|SET RAD TYPE | 11 | Configure RAD to either RAD_ARM or RAD_DRIVETRAIN, to configure default gear ratios and range of motions (value of TBD RAD ENUM)|
|SET HOME POSITION | 13 | Set number of whole rotation to return to after calibration (uint8)|
|SET ODOM INTERVAL | 15 | In ms (uint16_t) |
|SAVE TO EEPROM | 17 | Data ignored |
|SET HEALTH INTERVAL | 19 | In ms (uint16_t) |
|START CALIBRATE | 21 | Trigger Calibration Routine - ignore data|
|SET DRVCONF TST | 23 | uint8 value |
|SET DRVCONF SLP | 25 | uint8 value |
|SET DRVCONF DIS_S2G | 27 | uint8 value |
|SET DRVCONF TS2G | 29 | uint8 value |
|SET DRVCONF SDOFF | 31 | uint8 value |
|SET DRVCONF VSENSE | 33 | uint8 value|
| SET DRVCONF RDSEL | 35 | uint8 value |
|SET DRVCONF OTSENS | 37 | uint8 value |
|SET DRVCONF SHRTSENS | 39 | uint8 value |
|SET DRVCONF EN_PFD | 41 | uint8 value |
|SET DRVCONF EN_S2VS | 43 | uint8 value |
|SET SGCSCONF SFILT | 45 | uint8 value |
|SET SGCSCONF SGT | 47 | uint 8 value |
|SET SGCSCONF CS | 49 | uint 8 value 
|SET SMARTEN SEIMIN | 51 | uint 8 value 
|SET SMARTEN SEDN | 53 | uint 16 value 
|SET SMARTEN SEMAX | 55 | uint 8 value 
|SET SMARTEN SEUP | 57 | uint 8 value 
|SET SMARTEN SEMIN | 59 | uint 8 value 
|SET CHOPCONF TBL | 61 | uint 8 value 
|SET CHOPCONF CHM | 63 | uint 8 value 
|SET CHOPCONF RNDTF | 65 | uint 8 value 
|SET CHOPCONF HDEC | 67 | uint 8 value 
|SET CHOPCONF HEND | 69 | uint 8 value 
|SET CHOPCONF HSTRT | 71 | uint 8 value 
|SET CHOPCONF TOFF | 73 | uint 8 value 
|SET DRVCTRL INTPOL | 75 | uint 8 value 
|SET DRVCTRL DEDGE | 77 | uint 8 value 
|SET DRVCTRL MRES | 79 | uint 8 value 



## Status Commands (Outgoing)
| **Command Name** | **Command ID** | **Description**
|--------------|------------|-----------|
|SEND_ODOM_ANGLE | 251 |Limit Switch and Upper Bound State, Current Angle Reading (0 to 360 degrees) |
|SEND_HEALTH_STATUS | 252 | Byte 0: EEPROM status<br/> Byte 1: TMC status<br/> Byte 2: Encoder Status<br/> Byte 3: RAD State<br/> Byte 4: Limit Switch <br/>(big endian) |
