/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 MMRT.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at24c04c.h"
#include "tmc_2590.h"
#include "as5048a.h"
#include "pid.h"
#include "queue.h"
#include "enc_dec_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_PARAMS_EEPROM_PAGE 0
#define HARDSTOP_SAFETY_MARGIN 5
#define AVERAGING_WINDOW_SIZE 10

#define MOTOR_GEARING gearing
#define MAX_ROTATIONS max_rotations

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RAD_STATUS_TypeDef rad_status;
RAD_PARAMS_TypeDef rad_params;

uint8_t ESTOP = 0;
uint8_t DISABLED = 0;

uint16_t min_angle;
uint16_t max_angle;
uint16_t gearing;
uint16_t max_rotations;

float software_stop = 0;
uint8_t cw_enable = 0;
uint8_t ccw_enable = 0;

double* angle_average_buffer;
uint8_t buffer_head;

int16_t steps_to_move;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    //SET DEFAULT VALUES
    rad_params.RAD_ID = 0xF0;
    rad_params.RAD_TYPE = RAD_TYPE_UNDEFINED;
    rad_params.HOME_POSITION = RAD_TYPE_DRIVETRAIN_MAX_ROTATIONS/2;
    rad_params.STEPPER_SPEED = 500;
    rad_params.ODOM_INTERVAL = 20; //50hz, or 20ms
    rad_params.HEALTH_INTERVAL = 1000; //every second
    rad_params.P = 0.01;
    rad_params.I = 0.0000001;
    rad_params.D = 0;
    
    rad_params.CHOPCONF_CHM = 0b0;
    rad_params.CHOPCONF_HDEC = 0b00;
    rad_params.CHOPCONF_HEND = 0b0100;
    rad_params.CHOPCONF_HSTRT = 0b110;
    rad_params.CHOPCONF_RNDTF = 0b0;
    rad_params.CHOPCONF_TBL = 0b10;
    rad_params.CHOPCONF_TOFF = 0b100;

    rad_params.DRVCONF_DIS_S2G = 0b0;
    rad_params.DRVCONF_EN_PFD = 0b1;
    rad_params.DRVCONF_EN_S2VS = 0b1;
    rad_params.DRVCONF_OTSENS = 0b0;
    rad_params.DRVCONF_RDSEL = 0b11;
    rad_params.DRVCONF_SDOFF = 0b0;
    rad_params.DRVCONF_SHRTSENS = 0b1;
    rad_params.DRVCONF_SLP = 0b11110;
    rad_params.DRVCONF_TS2G = 0b00;
    rad_params.DRVCONF_TST = 0b0;
    rad_params.DRVCONF_VSENSE = 0b0;

    rad_params.DRVCTRL_DEDGE = 0b0;
    rad_params.DRVCTRL_INTPOL = 0b1;
    rad_params.DRVCTRL_MRES = 0b1000;

    rad_params.SGCSCONF_CS = 5;
    rad_params.SGCSCONF_SFILT = 0b0;
    rad_params.SGCSCONF_SGT = 0b0000010;

    rad_params.SMARTEN_SEDN = 0b00;
    rad_params.SMARTEN_SEIMIN = 0b0;
    rad_params.SMARTEN_SEMAX = 0b0000;
    rad_params.SMARTEN_SEMIN = 0b0000;
    rad_params.SMARTEN_SEUP = 0b00;


    //DRIVEDAY HARDCODE

    rad_params.RAD_ID = 0x14;
    rad_params.RAD_TYPE = RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT;
    rad_params.STEPPER_SPEED = 1000;
    rad_params.ODOM_INTERVAL = 20; //50hz, or 20ms
    rad_params.HEALTH_INTERVAL = 1000; //every second
    rad_params.P = 0.06;
    rad_params.I = 0.000001;
    rad_params.D = 0;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    //READ EEPROM HERE. Update rad_params if successful read


    MX_AT24C04C_1_Init(); 
    
    //uint8_t *temp = (uint8_t*) malloc(sizeof(RAD_PARAMS_TypeDef));

    //temporary so we don't cook stepper settings from reading garbage data from eeprom
    RAD_PARAMS_TypeDef eeprom_params;

    rad_status.EEPROM_STATUS = AT24C04C_ReadPages(&at24c04c_1, (uint8_t*)&eeprom_params, sizeof(RAD_PARAMS_TypeDef), RAD_PARAMS_EEPROM_PAGE);
    
    if (rad_status.EEPROM_STATUS == AT24C04C_OK)
    {
        //NORMAL OPERATION
        rad_params = eeprom_params;

        //IGNORE EEPROM AND SET DEFAULT PARAMS FOR FIRST EEPROM SAVE
        //rad_params.RAD_ID = 0x11;
//        rad_params.RAD_TYPE = RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT;
//        rad_params.ODOM_INTERVAL = 1000;
//        rad_params.HEALTH_INTERVAL = 5000;
        //memcpy(&backup, temp, sizeof(RAD_PARAMS_TypeDef));
    }
    //free(temp);



    MX_TMC_2590_1_Init();
    MX_AS5048A_1_Init();
    MX_PID_1_Init();


    switch(rad_params.RAD_TYPE)
    {
        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
        {
            MAX_ROTATIONS = RAD_TYPE_DRIVETRAIN_MAX_ROTATIONS;
            MOTOR_GEARING = RAD_TYPE_DRIVETRAIN_GEARING;
            break;
        }
        case RAD_TYPE_UNDEFINED:
        default:
        {
            MAX_ROTATIONS = 5; //60 degrees
            MOTOR_GEARING = RAD_TYPE_DRIVETRAIN_GEARING;
            break;
        }
    }

    min_angle = 0;
    max_angle = 360 * MAX_ROTATIONS / MOTOR_GEARING;

    angle_average_buffer = (double*) calloc(AVERAGING_WINDOW_SIZE, sizeof(double));
    
    MX_CAN_UpdateIdAndFilters(&rad_can);

    uint32_t arr = HAL_TIM_CalculateAutoReload(tmc_2590_1.Init.STEP_Tim, rad_params.STEPPER_SPEED);


    TMC_2590_SetTimAutoReload(&tmc_2590_1, arr);


    //SEND ERROR CODES OF EACH INIT MODULE
    MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    static enum 
    {
        RAD_STATE_INIT = 0,
        RAD_STATE_PULSE_CONTROL,
        RAD_STATE_CALIBRATION,
        RAD_STATE_ACTIVE
    } rad_state = RAD_STATE_INIT;

    while (1)
    {
        //Check and process global messages first
        if (!queue_empty(&can_message_queue_global))
        {
            RAD_CAN_Message_TypeDef *new_message =
                    (RAD_CAN_Message_TypeDef*) queue_front(
                            &can_message_queue_global);

            switch ((int)(new_message->command_id))
            {
                case ESTOP_MESSAGE:
                {
                    ESTOP = 1;
                    break;
                }
                case DISABLE_MESSAGE:
                {
                    DISABLED = 1;
                    break;
                }
                case ENABLE_MESSAGE:
                {
                    rad_state = RAD_STATE_PULSE_CONTROL;
                    DISABLED = 0;
                    break;
                }
                case HEALTH_STATUS_PING:
                {
                    MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);
                    break;
                }
                default:
                {
                    break;
                }
            }

            free(new_message->data);
            queue_dequeue(&can_message_queue_global);
        }
        //Only check rad queue after. This allows global messages to be addressed immediately
        else if (!queue_empty(&can_message_queue_rad))
        {
            RAD_CAN_Message_TypeDef *new_message =
                    (RAD_CAN_Message_TypeDef*) queue_front(
                            &can_message_queue_rad);

            switch ((int)(new_message->command_id))
            {
           
                case SET_TARGET_ANGLE:
                {
                    double new_setpoint = decode_double_big_endian(new_message->data);

                    if (new_setpoint < min_angle)
                    {
                        new_setpoint = min_angle;
                    }
                    else if (new_setpoint > max_angle)
                    {
                        new_setpoint = max_angle;
                    }
                    PID_ChangeSetPoint(&pid_1, new_setpoint*MOTOR_GEARING);
                    break;
                }
                case GET_ENCODER_VALUE:
                {
                    rad_status.current_angle = (double) (pid_1.feedback_adj / MOTOR_GEARING); //skip the buffer
                    MX_CAN_Broadcast_Odometry_Message(&rad_can, rad_status);
                    break;
                }
                case SET_STEPPER_SPEED:
                {
                    //1 calculate ARR from inputted desired freq
                    //Will be an integer floor divide so will not always be the same as input

                	uint32_t arr = HAL_TIM_CalculateAutoReload(tmc_2590_1.Init.STEP_Tim,
                        decode_uint32_big_endian(new_message->data));

                    //2 assign ARR to timer

                    TMC_2590_SetTimAutoReload(&tmc_2590_1, arr);

                    //3 update local stepper speed reference to the integer value

                    rad_params.STEPPER_SPEED = HAL_TIM_CalculateFrequency(tmc_2590_1.Init.STEP_Tim);

                    break;
                }
                case GET_STEPPER_SPEED:
                {
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.STEPPER_SPEED, GET_STEPPER_SPEED);
                    break;
                }
                case SET_P_VALUE:
                {
                    pid_1.Init.kp = decode_double_big_endian(new_message->data);
                    rad_params.P = pid_1.Init.kp;
                    break;
                }
                case GET_P_VALUE:
                {
                    MX_CAN_Broadcast_Double_Data(&rad_can, pid_1.Init.kp, GET_P_VALUE);
                    break;
                }
                case SET_I_VALUE:
                {
                    pid_1.Init.ki = decode_double_big_endian(new_message->data);
                    rad_params.I = pid_1.Init.ki;
                    break;
                }
                case GET_I_VALUE:
                {
                    MX_CAN_Broadcast_Double_Data(&rad_can, pid_1.Init.ki, GET_I_VALUE);
                    break;
                }
                case SET_D_VALUE:
                {
                    pid_1.Init.kd = decode_double_big_endian(new_message->data);
                    rad_params.D = pid_1.Init.kd;
                    break;
                }
                case GET_D_VALUE:
                {
                    MX_CAN_Broadcast_Double_Data(&rad_can, pid_1.Init.kd, GET_D_VALUE);
                    break;
                }
                case SET_RAD_TYPE:
                {
                    rad_params.RAD_TYPE = new_message->data[0];
                    break;
                }
                case GET_RAD_TYPE:
                {
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.RAD_TYPE, GET_RAD_TYPE);
                    break;
                }
                case SET_HOME_POSITION:
                {
                    rad_params.HOME_POSITION = decode_uint32_big_endian(new_message->data);
                    break;
                }
                case GET_HOME_POSITION:
                {
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.HOME_POSITION, GET_HOME_POSITION);
                    break;
                } 
                case SET_ODOM_INTERVAL:
                {
                    rad_params.ODOM_INTERVAL = decode_uint32_big_endian(new_message->data);
                    break;
                }
                case GET_ODOM_INTERVAL:
                {
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.ODOM_INTERVAL, GET_ODOM_INTERVAL);
                    break;
                }
                case SAVE_TO_EEPROM:
                {
                    AT24C04C_WritePages(&at24c04c_1, (uint8_t*)&rad_params, sizeof(RAD_PARAMS_TypeDef), RAD_PARAMS_EEPROM_PAGE);
                    break;
                }
                case RELOAD_FROM_EEPROM:
                {
                    AT24C04C_ReadPages(&at24c04c_1, (uint8_t*)&rad_params, sizeof(RAD_PARAMS_TypeDef), RAD_PARAMS_EEPROM_PAGE);
                    break;
                }
                case SET_HEALTH_INTERVAL:
                {
                    rad_params.HEALTH_INTERVAL = decode_uint32_big_endian(new_message->data);
                    break;
                }
                case GET_HEALTH_INTERVAL:
                {
                    MX_CAN_Broadcast_Uint32_Data(&rad_can, rad_params.HEALTH_INTERVAL, GET_HEALTH_INTERVAL);
                    break;
                } 
                case START_CALIBRATE:
                {
                    rad_state = RAD_STATE_CALIBRATION;
                    break;
                }
                case CANCEL_CALIBRATION:
                {
                    //cancel mid calibration OR ignore calibrated params and return to pulse
                    rad_state = RAD_STATE_INIT;
                    break;
                }
                case SET_DRVCONF_TST:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.tst = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_TST = tmc_2590_1.ConfRegisters.DRVCONF.tst;
                    break;
                }
                case GET_DRVCONF_TST:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.tst, GET_DRVCONF_TST);
                    break;
                }
                case SET_DRVCONF_SLP:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.slp = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_SLP = tmc_2590_1.ConfRegisters.DRVCONF.slp;
                    break;
                }
                case GET_DRVCONF_SLP:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.slp, GET_DRVCONF_SLP);
                    break;
                }
                case SET_DRVCONF_DIS_S2G:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.dis_s2g = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_DIS_S2G= tmc_2590_1.ConfRegisters.DRVCONF.dis_s2g;
                    break;
                }
                case GET_DRVCONF_DIS_S2G:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.dis_s2g, GET_DRVCONF_DIS_S2G);
                    break;
                }
                case SET_DRVCONF_TS2G:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.ts2g = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_TS2G = tmc_2590_1.ConfRegisters.DRVCONF.ts2g;
                    break;
                }
                case GET_DRVCONF_TS2G:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.ts2g, GET_DRVCONF_TS2G);
                    break;
                }
                case SET_DRVCONF_SDOFF:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.sdoff = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_SDOFF = tmc_2590_1.ConfRegisters.DRVCONF.sdoff;
                    break;
                }
                case GET_DRVCONF_SDOFF:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.sdoff, GET_DRVCONF_SDOFF);
                    break;
                }
                case SET_DRVCONF_VSENSE:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.vsense = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_VSENSE = tmc_2590_1.ConfRegisters.DRVCONF.vsense;
                    break;
                }
                case GET_DRVCONF_VSENSE:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.vsense, GET_DRVCONF_VSENSE);
                    break;
                }
                case SET_DRVCONF_RDSEL:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.rdsel = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_RDSEL = tmc_2590_1.ConfRegisters.DRVCONF.rdsel;
                    break;
                }
                case GET_DRVCONF_RDSEL:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.rdsel, GET_DRVCONF_RDSEL);
                    break;
                }
                case SET_DRVCONF_OTSENS:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.otsens = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_OTSENS = tmc_2590_1.ConfRegisters.DRVCONF.otsens;
                    break;
                }
                case GET_DRVCONF_OTSENS:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.otsens, GET_DRVCONF_OTSENS);
                    break;
                }
                case SET_DRVCONF_SHRTSENS:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.shrtsens = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_SHRTSENS = tmc_2590_1.ConfRegisters.DRVCONF.shrtsens;
                    break;
                }
                case GET_DRVCONF_SHRTSENS:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.shrtsens, GET_DRVCONF_SHRTSENS);
                    break;
                }
                case SET_DRVCONF_EN_PFD:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.en_pfd = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_EN_PFD = tmc_2590_1.ConfRegisters.DRVCONF.en_pfd;
                    break;
                }
                case GET_DRVCONF_EN_PFD:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.en_pfd, GET_DRVCONF_EN_PFD);
                    break;
                }
                case SET_DRVCONF_EN_S2VS:
                {
                    tmc_2590_1.ConfRegisters.DRVCONF.en_s2vs = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCONF_EN_S2VS = tmc_2590_1.ConfRegisters.DRVCONF.en_s2vs;
                    break;
                }
                case GET_DRVCONF_EN_S2VS:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCONF.en_s2vs, GET_DRVCONF_EN_S2VS);
                    break;
                }
                case SET_SGCSCONF_SFILT:
                {
                    tmc_2590_1.ConfRegisters.SGCSCONF.sfilt = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SGCSCONF_SFILT = tmc_2590_1.ConfRegisters.SGCSCONF.sfilt;
                    break;
                }
                case GET_SGCSCONF_SFILT:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SGCSCONF.sfilt, GET_SGCSCONF_SFILT);
                    break;
                }
                case SET_SGCSCONF_SGT:
                {
                    tmc_2590_1.ConfRegisters.SGCSCONF.sgt = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SGCSCONF_SGT = tmc_2590_1.ConfRegisters.SGCSCONF.sgt;
                    break;
                }
                case GET_SGCSCONF_SGT:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SGCSCONF.sgt, GET_SGCSCONF_SGT);
                    break;
                }
                case SET_SGCSCONF_CS:
                {
                    tmc_2590_1.ConfRegisters.SGCSCONF.cs = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SGCSCONF_CS = tmc_2590_1.ConfRegisters.SGCSCONF.cs;
                    break;
                }
                case GET_SGCSCONF_CS:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SGCSCONF.cs, GET_SGCSCONF_CS);
                    break;
                }
                case SET_SMARTEN_SEIMIN:
                {
                    tmc_2590_1.ConfRegisters.SMARTEN.seimin = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SMARTEN_SEIMIN = tmc_2590_1.ConfRegisters.SMARTEN.seimin;
                    break;
                }
                case GET_SMARTEN_SEIMIN:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SMARTEN.seimin, GET_SMARTEN_SEIMIN);
                    break;
                }
                case SET_SMARTEN_SEDN:
                {
                    tmc_2590_1.ConfRegisters.SMARTEN.sedn = decode_uint16_big_endian(new_message->data); //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SMARTEN_SEDN = tmc_2590_1.ConfRegisters.SMARTEN.sedn;
                    break;
                }
                case GET_SMARTEN_SEDN:
                {
                    MX_CAN_Broadcast_Uint16_Data(&rad_can, tmc_2590_1.ConfRegisters.SMARTEN.sedn, GET_SMARTEN_SEDN);
                    break;
                }
                case SET_SMARTEN_SEMAX:
                {
                    tmc_2590_1.ConfRegisters.SMARTEN.semax = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SMARTEN_SEMAX = tmc_2590_1.ConfRegisters.SMARTEN.semax;
                    break;
                }
                case GET_SMARTEN_SEMAX:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SMARTEN.semax, GET_SMARTEN_SEMAX);
                    break;
                }
                case SET_SMARTEN_SEUP:
                {
                    tmc_2590_1.ConfRegisters.SMARTEN.seup = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SMARTEN_SEUP = tmc_2590_1.ConfRegisters.SMARTEN.seup;
                    break;
                }
                case GET_SMARTEN_SEUP:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SMARTEN.seup, GET_SMARTEN_SEUP);
                    break;
                }
                case SET_SMARTEN_SEMIN:
                {
                    tmc_2590_1.ConfRegisters.SMARTEN.semin = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.SMARTEN_SEMIN = tmc_2590_1.ConfRegisters.SMARTEN.semin;
                    break;
                }
                case GET_SMARTEN_SEMIN:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.SMARTEN.semin, GET_SMARTEN_SEMIN);
                    break;
                }
                case SET_CHOPCONF_TBL:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.tbl = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_TBL = tmc_2590_1.ConfRegisters.CHOPCONF.tbl;
                    break;
                }
                case GET_CHOPCONF_TBL:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.tbl, GET_CHOPCONF_TBL);
                    break;
                }
                case SET_CHOPCONF_CHM:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.chm = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_CHM = tmc_2590_1.ConfRegisters.CHOPCONF.chm;
                    break;
                }
                case GET_CHOPCONF_CHM:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.chm, GET_CHOPCONF_CHM);
                    break;
                }
                case SET_CHOPCONF_RNDTF:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.rndtf = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_RNDTF = tmc_2590_1.ConfRegisters.CHOPCONF.rndtf;
                    break;
                }
                case GET_CHOPCONF_RNDTF:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.rndtf, GET_CHOPCONF_RNDTF);
                    break;
                }
                case SET_CHOPCONF_HDEC:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.hdec = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_HDEC = tmc_2590_1.ConfRegisters.CHOPCONF.hdec;
                    break;
                }
                case GET_CHOPCONF_HDEC:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.hdec, GET_CHOPCONF_HDEC);
                    break;
                }
                case SET_CHOPCONF_HEND:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.hend = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_HEND = tmc_2590_1.ConfRegisters.CHOPCONF.hend;
                    break;
                }
                case GET_CHOPCONF_HEND:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.hend, GET_CHOPCONF_HEND);
                    break;
                }
                case SET_CHOPCONF_HSTRT:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.hstrt = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_HSTRT = tmc_2590_1.ConfRegisters.CHOPCONF.hstrt;
                    break;
                }
                case GET_CHOPCONF_HSTRT:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.hstrt, GET_CHOPCONF_HSTRT);
                    break;
                }
                case SET_CHOPCONF_TOFF:
                {
                    tmc_2590_1.ConfRegisters.CHOPCONF.toff = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.CHOPCONF_TOFF = tmc_2590_1.ConfRegisters.CHOPCONF.toff;
                    break;
                }
                case GET_CHOPCONF_TOFF:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.CHOPCONF.toff, GET_CHOPCONF_TOFF);
                    break;
                }
                case SET_DRVCTRL_INTPOL:
                {
                    tmc_2590_1.ConfRegisters.DRVCTRL.intpol = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCTRL_INTPOL = tmc_2590_1.ConfRegisters.DRVCTRL.intpol;
                    break;
                }
                case GET_DRVCTRL_INTPOL:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCTRL.intpol, GET_DRVCTRL_INTPOL);
                    break;
                }
                case SET_DRVCTRL_DEDGE:
                {
                    tmc_2590_1.ConfRegisters.DRVCTRL.dedge = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCTRL_DEDGE = tmc_2590_1.ConfRegisters.DRVCTRL.dedge;
                    break;
                }
                case GET_DRVCTRL_DEDGE:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCTRL.dedge, GET_DRVCTRL_DEDGE);
                    break;
                }
                case SET_DRVCTRL_MRES:
                {
                    tmc_2590_1.ConfRegisters.DRVCTRL.mres = new_message->data[0]; //uint8, big endian
                    rad_status.TMC_STATUS = TMC_2590_WriteConfRegisters(&tmc_2590_1);
                    rad_params.DRVCTRL_MRES = tmc_2590_1.ConfRegisters.DRVCTRL.mres;
                    break;
                }
                case GET_DRVCTRL_MRES:
                {
                    MX_CAN_Broadcast_Uint8_Data(&rad_can, tmc_2590_1.ConfRegisters.DRVCTRL.mres, GET_DRVCTRL_MRES);
                    break;
                }
                case PULSE_STEPPER:
                {
                    float pulses = decode_float_big_endian(new_message->data);
                    steps_to_move = (int16_t) pulses;

                    //cap steps
                    if (steps_to_move > tmc_2590_1.Init.max_steps)
                    {
                        steps_to_move = tmc_2590_1.Init.max_steps;
                    }
                    else if (steps_to_move < -1*(tmc_2590_1.Init.max_steps))
                    {
                        steps_to_move = -1*tmc_2590_1.Init.max_steps;
                    }
                    break;
                }
                case REBOOT:
                {
                	HAL_NVIC_SystemReset();
                    break;
                }
                case ASSIGN_DEVICE_ID:
                {
                    rad_can.id = new_message->data[0];
                    rad_params.RAD_ID = rad_can.id;

                    MX_CAN_UpdateIdAndFilters(&rad_can);

                    break;
                }
                default:
                    break;
            }
            free(new_message->data);
            queue_dequeue(&can_message_queue_rad);

        }

        //CHECK FOR ESTOP
        if (ESTOP)
        {
        	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            break;
        }
        if (DISABLED)
        {
        	steps_to_move = 0;
        	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            continue;
        }
        else
        {
        	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
        }

        

        switch (rad_state)
        {

            case RAD_STATE_INIT:
            {
                //init stuff here
                rad_state = RAD_STATE_PULSE_CONTROL;
                break;
            }
            case RAD_STATE_PULSE_CONTROL:
            {
                GPIO_PinState ls_state = HAL_GPIO_ReadPin(LS_1_GPIO_Port, LS_1_Pin);
                rad_status.ls_1 = ls_state;

                cw_enable = 1;
        	    ccw_enable = 1;

                if (ls_state == GPIO_PIN_SET)
                {
                    switch (rad_params.RAD_TYPE) 
                    {

                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
                        {
                            cw_enable = 0;
                            ccw_enable = 1;
                            break;
                        }
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
                        {
                            cw_enable = 1;
                            ccw_enable = 0;
                            break;
                        }
                        default:
                            break;
				    }
                }

                if ((steps_to_move > 0) && cw_enable)
                {
                    rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, steps_to_move);
                }
                else if ((steps_to_move < 0) && ccw_enable)
                {
                    rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, steps_to_move);
                }

                steps_to_move = 0;

                break;
            }
            case RAD_STATE_CALIBRATION:
            {

                GPIO_PinState ls_state = HAL_GPIO_ReadPin(LS_1_GPIO_Port, LS_1_Pin);
                rad_status.ls_1 = ls_state;
                
                if (ls_state == GPIO_PIN_SET)
                {
                    switch (rad_params.RAD_TYPE) 
                    {
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
                            PID_SetMaxPoint(&pid_1, MAX_ROTATIONS);
                            PID_ChangeSetPoint(&pid_1, max_angle*MOTOR_GEARING);
                            software_stop = min_angle;
                            break;
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
                            PID_SetZeroPoint(&pid_1);
                            PID_ChangeSetPoint(&pid_1, min_angle*MOTOR_GEARING);
                            software_stop = max_angle;
                            break;
                        default:
                            break;
                    }

                    PID_Update(&pid_1);
                    PID_Update(&pid_1);
                    PID_Update(&pid_1);

                    

                    //PID_Update(&pid_1);

                    rad_state = RAD_STATE_ACTIVE;
        	    }
                else
                {
                    switch (rad_params.RAD_TYPE)
                    {
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
                        {
                            rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, 50);
                            break;
                        }
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
                        {
                            rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, -50);
                            break;
                        }
                    
                    default:
                        break;
                    }
                }

                break;
            }
            case RAD_STATE_ACTIVE:
            {

                cw_enable = 1;
        	    ccw_enable = 1;

                GPIO_PinState ls_state = HAL_GPIO_ReadPin(LS_1_GPIO_Port, LS_1_Pin);
                rad_status.ls_1 = ls_state;
        
                uint8_t i = 0; //safety limit
                static uint8_t consecutive_encoder_failures = 0;
                while ((rad_status.ENCODER_STATUS = AS5048A_ReadAngle(&as5048a_1)) != AS5048A_OK)
                {
                    if (i++ > 10)
                    {
                        break;
                    } 
                }

                if (rad_status.ENCODER_STATUS != AS5048A_OK)
                {
                    if (consecutive_encoder_failures++ > 3)
                    {
                         //encoder has failed
                        rad_state = RAD_STATE_PULSE_CONTROL;
                    }
                }   
                else
                {
                    //reset failure counter
                    consecutive_encoder_failures = 0;
                }

                
                PID_Update(&pid_1);

                if (ls_state == GPIO_PIN_SET)
                {
                    switch (rad_params.RAD_TYPE) 
                    {

                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
                        {
                            PID_SetMaxPoint(&pid_1, MAX_ROTATIONS);
                            cw_enable = 0;
                            ccw_enable = 1;
                            break;
                        }
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
                        {
                            PID_SetZeroPoint(&pid_1);
                            cw_enable = 1;
                            ccw_enable = 0;
                            break;
                        }
                        default:
                            break;
				    }

                }
                else 
                {
                    switch (rad_params.RAD_TYPE)
                    {
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_RIGHT:
                        {
                            if ((pid_1.feedback_adj / MOTOR_GEARING) <= (software_stop + HARDSTOP_SAFETY_MARGIN)) 
                            {
                                cw_enable = 1;
                                ccw_enable = 0;
                                PID_ClearIError(&pid_1);
                            }
                            break;
                        }
                        case RAD_TYPE_DRIVETRAIN_LIMIT_SWITCH_LEFT:
                        {
                            if ((pid_1.feedback_adj / MOTOR_GEARING) >= (software_stop - HARDSTOP_SAFETY_MARGIN)) 
                            {
                                cw_enable = 0;
                                ccw_enable = 1;
                                PID_ClearIError(&pid_1);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }

                if ((pid_1.output > 0) && cw_enable)
                {
                    rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, (int16_t) pid_1.output);
                }
                else if ((pid_1.output < 0) && ccw_enable)
                {
                    rad_status.TMC_STATUS = TMC_2590_MoveSteps(&tmc_2590_1, (int16_t) pid_1.output);
                }

                break;
            }
            default:
            {
                rad_state = RAD_STATE_INIT;
                break;
            }
        }

        //angle_average_buffer[buffer_head++ % AVERAGING_WINDOW_SIZE] = (double) (pid_1.feedback_adj / MOTOR_GEARING);

                
        if ((rad_params.ODOM_INTERVAL != 0) && (HAL_GetTick() % rad_params.ODOM_INTERVAL == 0))
        {
//             double sum = 0;
//             for (int i = 0; i < AVERAGING_WINDOW_SIZE; i++)
//             {
//                 sum = sum + angle_average_buffer[i];
//             }
            //rad_status.current_angle = (double) sum / AVERAGING_WINDOW_SIZE;
            rad_status.current_angle = (double) (pid_1.feedback_adj / MOTOR_GEARING);
        	//rad_status.current_angle = (double) pid_1.output;
            //AS5048A_ReadAngle(&as5048a_1);
            //rad_status.current_angle = as5048a_1.Angle_double;

            MX_CAN_Broadcast_Odometry_Message(&rad_can, rad_status);
        }
        
        if ((rad_params.HEALTH_INTERVAL != 0) && (HAL_GetTick() % rad_params.HEALTH_INTERVAL == 0))
        {
            rad_status.RAD_STATE = rad_state;
            MX_CAN_Broadcast_Health_Message(&rad_can, rad_status);
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    TMC_2590_TIM_PWM_PulseFinishedCallback(&tmc_2590_1, htim);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
