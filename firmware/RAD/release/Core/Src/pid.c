/*
 * pid.c
 *
 *  Created on: Jun 23, 2024
 *      Author: Ethan
 */

#include "pid.h"
#include "as5048a.h"
#include "at24c04c.h"
#include "enc_dec_utils.h"

PID_HandleTypeDef pid_1;

void MX_PID_1_Init(void)
{
    uint8_t eeprom_buff[8];
    pid_1.Init.feedback = &(as5048a_1.Angle_double);

    // todo pull from eeprom
//    AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_P_VALUE, eeprom_buff,
//            sizeof(double));
//    pid_1.Init.kp = *((double*) eeprom_buff);
//    AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_I_VALUE, eeprom_buff,
//            sizeof(double));
//    pid_1.Init.ki = *((double*) eeprom_buff);
//    AT24C04C_ReadData(&at24c04c_1, EEPROM_ADDR_D_VALUE, eeprom_buff,
//            sizeof(double));
//    pid_1.Init.kd = *((double*) eeprom_buff);

    pid_1.Init.kp = 0.06;
    pid_1.Init.ki = 0.0001;
    pid_1.Init.kd = 0;

    pid_1.Init.max_output_abs = 1000.0;
    pid_1.Init.rollover_max = 360.0;

    PID_Init(&pid_1);

//    if (PID_Init(&pid_1) != PID_OK)
//    {
//        Error_Handler();
//    }

    PID_Update(&pid_1);
    PID_Update(&pid_1);
    PID_Update(&pid_1);
    // set startup as zero point
    PID_SetZeroPoint(&pid_1);
    PID_ChangeSetPoint(&pid_1, 0.0);
    PID_Update(&pid_1);
}

