/*
 * ina238.h
 *
 *  Created on: Jan 10, 2025
 *      Author: ishan
 */

#ifndef INA238_INC_INA238_H_
#define INA238_INC_INA238_H_

//Define register values

typedef enum
{
    INA_238_OK = 0x00U,
    INA_238_ERROR = 0x01U,
    INA_238_BUSY = 0x02U,
    INA_238_TIMEOUT = 0x03U
} INA_238_StatusTypeDef;


typedef struct
{
    //register params
} INA_238_ConfRegister_TypeDef

typedef struct
{
	//Enter key params here

    //state

    //alert pin
    //i2c instance
    //shunt resistor value

    //register instance

    //voltage reading
    //current reading
    //power reading
    //diagnostic reading
} INA_238_HandleTypeDef;

INA_238_StatusTypeDef INA_238_Init(INA_238_HandleTypeDef* ina_238);

INA_238_StatusTypeDef INA_238_DeInit(INA_238_HandleTypeDef* ina_238);

INA_238_StatusTypeDef INA_238_ReadCurrent(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadVoltage(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadPower(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_ReadDiagnostic(INA_238_HandleTypeDef* ina_238);
INA_238_StatusTypeDef INA_238_WriteConfig(INA_238_HandleTypeDef* ina_238);

uint16_t __i2c_read(INA_238_HandleTypeDef* ina_238, uint16_t address, uint8_t *buffer, uint16_t len_bytes);

uint16_t __i2c_write(INA_238_HandleTypeDef* ina_238, uint16_t address, uint8_t *data, uint16_t len_bytes);




#endif /* INA238_INC_INA238_H_ */
