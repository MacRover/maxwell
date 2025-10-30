#ifndef _SCIENCE_H
#define _SCIENCE_H

#include "DFRobot_MultiGasSensor.h" //Have to download from https://github.com/DFRobot/DFRobot_MultiGasSensor/releases/tag/V3.0.0 
// #include "DFRobot_OzoneSensor.h" // https://github.com/DFRobot/DFRobot_OzoneSensor/releases/tag/V1.0.1 

#include "enums.h"
#include "i2c.h"

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#define I2C_ADDRESS_HYDROGEN_SENSOR 0x74
#define I2C_ADDRESS_OZONE_SENSOR 0x73

#define           OZONE_ADDRESS_0           0x70
#define           OZONE_ADDRESS_1           0x71
#define           OZONE_ADDRESS_2           0x72
#define           OZONE_ADDRESS_3           0x73
#define           MEASURE_MODE_AUTOMATIC    0x00           ///< active  mode
#define           MEASURE_MODE_PASSIVE      0x01           ///< passive mode
#define           AUTO_READ_DATA            0x00           ///< auto read ozone data
#define           PASSIVE_READ_DATA         0x01           ///< passive read ozone data
#define           MODE_REGISTER             0x03           ///< mode register
#define           SET_PASSIVE_REGISTER      0x04           ///< read ozone data register
#define           AUTO_DATA_HIGE_REGISTER   0x09           ///< AUTO data high eight bits
#define           AUTO_DATA_LOW_REGISTER    0x0A           ///< AUTO data Low  eight bits
#define           PASS_DATA_HIGE_REGISTER   0x07           ///< AUTO data high eight bits
#define           PASS_DATA_LOW_REGISTER    0x08           ///< AUTO data Low  eight bits
#define           OCOUNT                    100            ///< Ozone Count Value

std_msgs__msg__Float32 hydrogen_msg;
std_msgs__msg__Int16 ozone_msg;

DFRobot_GAS_I2C hydrogen_sensor(&Wire1, I2C_ADDRESS_HYDROGEN_SENSOR);


void update_hydrogen_message(float val)
{
    hydrogen_msg.data = val;
}

void update_ozone_message(int16_t val)
{
    ozone_msg.data = val;
}

class DFRobot_OzoneSensor{
public:
  DFRobot_OzoneSensor(TwoWire *pWire = &Wire)
  {
    this->_pWire = pWire;
  }
  ~DFRobot_OzoneSensor()
  {
    this->_pWire = NULL;
  }

  /**
   * @fn begin
   * @brief initialization function
   * @param i2c address
   * @n     OZONE_ADDRESS_0  0x70
   * @n     OZONE_ADDRESS_1  0x71
   * @n     OZONE_ADDRESS_2  0x72
   * @n     OZONE_ADDRESS_3  0x73
   * @return bool type
   * @retval true initialization success
   * @retval false initialization failed
   */
  bool begin(uint8_t addr = OZONE_ADDRESS_0)
  {
    this->_addr = addr;
    _pWire->begin();
    _pWire->beginTransmission(_addr);
    if(_pWire->endTransmission() == 0){
        return true;
    }
    return false;
  }

  /**
   * @fn setModes
   * @brief Set mode Active or passive measurement.
   * @param mode mode is Active or passive.
   * @n       MEASURE_MODE_AUTOMATIC    active  mode
   * @n       MEASURE_MODE_PASSIVE      passive mode
   * @return None
   */
  void setModes(uint8_t mode)
  {
    if(mode == MEASURE_MODE_AUTOMATIC){
        write_i2c_8bit(this->_addr, MODE_REGISTER , MEASURE_MODE_AUTOMATIC);
        _M_Flag = 0;
    }else if(mode == MEASURE_MODE_PASSIVE){
        write_i2c_8bit(this->_addr, MODE_REGISTER , MEASURE_MODE_PASSIVE);
        _M_Flag = 1;
    }else {
        return;
    }
  }

  /**
   * @fn readOzoneData
   * @brief read ozone data.
   * @param collectNum is the number of data collected,(The default value is 20)
   * @n       COLLECT_NUMBER       The collection range is 1-100
   * @return ozone concentration: one part per billion (PPB).
   */
  int16_t readOzoneData(uint8_t collectNum = 20)
  {
    static uint8_t i = 0, j = 0;
    if (collectNum > 0) {
        for(j = collectNum - 1;  j > 0; j--){
        ozoneData[j] = ozoneData[j-1];
        }
        if(_M_Flag == 0){
            write_i2c_8bit(this->_addr, SET_PASSIVE_REGISTER , AUTO_READ_DATA);
            ozoneData[0] = i2cReadOzoneData(AUTO_DATA_HIGE_REGISTER);
        }else if(_M_Flag == 1){
            write_i2c_8bit(this->_addr, SET_PASSIVE_REGISTER , PASSIVE_READ_DATA);
            ozoneData[0] = i2cReadOzoneData(PASS_DATA_HIGE_REGISTER);
        }
        if(i < collectNum) i++;
        return getAverageNum(ozoneData, i);
    }else if(collectNum <= 0 || collectNum > OCOUNT){
        return -1;
    }
    return 0;
  }
  
  
private:
  int16_t i2cReadOzoneData(uint8_t reg)
  {
    uint8_t i = 0;
    uint16_t res;
    read_i2c_16bit(this->_addr, reg, &res);
    return (int16_t)res;
  }

  int getAverageNum(int bArray[], int iFilterLen)
  {
    unsigned long bTemp = 0;
    for(uint16_t i = 0; i < iFilterLen; i++) {
        bTemp += bArray[i];
    }
    return bTemp / iFilterLen;
  }

  int ozoneData[OCOUNT] = {0x00};
  uint8_t _addr;       ///< IIC Slave number
  uint8_t _M_Flag = 0; ///< mode flag
  TwoWire *_pWire;
};

DFRobot_OzoneSensor ozone_sensor(&Wire1);

#endif