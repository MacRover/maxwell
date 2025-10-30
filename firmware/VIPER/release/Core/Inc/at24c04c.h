/*
 * at24c04c.h
 *
 *  Created on: Jul 30, 2024
 *      Author: Ethan
 */

#ifndef INC_AT24C04C_H_
#define INC_AT24C04C_H_

#include <stm32f1xx_at24c04c.h>

#define EEPROM_ADDR_CAN_ID 0x010
#define EEPROM_ADDR_CW_DIR 0x011 // 1 if positive direction is clockwise
#define EEPROM_ADDR_OUTPUT_RATIO 0x020
#define EEPROM_ADDR_P_VALUE 0x060
#define EEPROM_ADDR_I_VALUE 0x0A0
#define EEPROM_ADDR_D_VALUE 0x0E0
//#define EEPROM_ADDR_TMC2590_CHOPCONF
//#define EEPROM_ADDR_TMC2590_DRVCONF
//#define EEPROM_ADDR_TMC2590_DRVCTRL
//#define EEPROM_ADDR_TMC2590_SGCSCONF
//#define EEPROM_ADDR_TMC2590_SMARTEN

extern AT24C04C_HandleTypeDef at24c04c_1;

void MX_AT24C04C_1_Init(void);


#endif /* INC_AT24C04C_H_ */
