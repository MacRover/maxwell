#include <micro_ros_arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <LSM6DSRSensor.h>
#include <Servo.h>
#include <RadioLib.h>

#include <cstdint>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>


#include "servo.h"
#include "science.h"
#include "viper_topics.h"
#include "led.h"
#include "gps.h"
#include "cooling.h"
#include "health.h"
#include "enums.h"
// #define ON_ROVER
#define USING_ROS
#define USING_LED
#define USING_IMU_ONBOARD
// #define USING_IMU_OTHER
//#define USING_TSB
// #define USING_FANS
// #define USING_SERVO
// #define USING_SCIENCE_SENSORS
// #define USING_LORA

// --- STATE VARIABLES ---
UROS_states state_UROS;
HYDROGEN_STATES state_hydrogen;
OZONE_STATES state_ozone;
LORA_STATES state_lora;


#define DOMAIN_ID 5

#define LED_PIN 13
#define AD0_VAL 1
#define IMU_INT1 23
#define MG_TO_MS2 0.0098066
#define DEG_TO_RAD 0.01745329 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define ROS_EXECUTE_INTERVAL(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t teensy_node;

rcl_publisher_t imu_pub;
rcl_publisher_t gps_pub;
rcl_publisher_t tsb_pub;
rcl_publisher_t hydrogen_pub;
rcl_publisher_t ozone_pub;
rcl_publisher_t health_pub;

sensor_msgs__msg__Imu imu_msg;

LSM6DSRSensor LSM6DSMR(&Wire1, LSM6DSR_I2C_ADD_H);
ICM_20948_I2C ICM;
#ifdef USING_LORA
SX1262 radio = new Module(10, 32, 40, 39); // CS, DIO1, NRST, BUSY

int16_t Transmission_State = RADIOLIB_ERR_NONE;


volatile bool Transmitted_Flag = false;

  #if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
  #endif
void Packet_Sent() {
  Transmitted_Flag = true;
}
#endif

uint8_t arduino_mac[] = { 0x04, 0xE9, 0xE5, 0x13, 0x0E, 0x4B };
IPAddress arduino_ip(192, 168, 1, 177);
#ifdef ON_ROVER
    IPAddress agent_ip(192, 168, 1, 111);
#else
    IPAddress agent_ip(192, 168, 1, 199);
#endif

unsigned long prev_time1 = 0, prev_time2 = 0, prev_time_lora, prev_time_hydrogen, prev_time_ozone = 0;
rcl_init_options_t init_options;


static struct micro_ros_agent_locator locator;



void updateICM_20948(ICM_20948_I2C* icm)
{
    imu_msg.linear_acceleration.x = icm->accX() * MG_TO_MS2;
    imu_msg.linear_acceleration.y = icm->accY() * MG_TO_MS2;
    imu_msg.linear_acceleration.z = icm->accZ() * MG_TO_MS2;

    imu_msg.angular_velocity.x = icm->gyrX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = icm->gyrY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = icm->gyrZ() * DEG_TO_RAD;

    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;
}

void updateLSM6DSM(LSM6DSRSensor* sensor)
{
    int32_t accel[3];
    int32_t gyro[3];

    if (sensor->Get_X_Axes(accel) == LSM6DSR_OK)
    {
        imu_msg.linear_acceleration.x = (double)(accel[0]) * MG_TO_MS2;
        imu_msg.linear_acceleration.y = (double)(accel[1]) * MG_TO_MS2;
        imu_msg.linear_acceleration.z = (double)(accel[2]) * MG_TO_MS2;
    }

    if (sensor->Get_G_Axes(gyro) == LSM6DSR_OK)
    {
        imu_msg.angular_velocity.x = (double)(gyro[0]) * (DEG_TO_RAD / 1000.0);
        imu_msg.angular_velocity.y = (double)(gyro[1]) * (DEG_TO_RAD / 1000.0);
        imu_msg.angular_velocity.z = (double)(gyro[2]) * (DEG_TO_RAD / 1000.0);
    }

    imu_msg.orientation_covariance[0] = -1;
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;
}

void obc_destory_uros_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&gps_pub, &teensy_node);
    rcl_publisher_fini(&imu_pub, &teensy_node);
    rcl_publisher_fini(&tsb_pub, &teensy_node);
    rcl_publisher_fini(&hydrogen_pub, &teensy_node);
    rcl_publisher_fini(&ozone_pub, &teensy_node);
    rcl_publisher_fini(&health_pub, &teensy_node);
    rcl_subscription_fini(&servo1_sub, &teensy_node);
    rcl_subscription_fini(&servo2_sub, &teensy_node);

    #ifdef USING_LED
    rcl_subscription_fini(&led_sub, &teensy_node);
    rclc_executor_fini(&led_executor);
    #endif

    destroy_viper_topics(&teensy_node);
    rcl_node_fini(&teensy_node);
    rclc_support_fini(&support);
}
static bool options_initialized = false;
bool obc_setup_uros()
{
#ifdef USING_ROS
    if (Ethernet.linkStatus() == LinkOFF) {
    return false;
  }

  locator.address = agent_ip;
  locator.port = 9999;

  RCCHECK(rmw_uros_set_custom_transport(
    false,
    (void *) &locator,
    arduino_native_ethernet_udp_transport_open,
    arduino_native_ethernet_udp_transport_close,
    arduino_native_ethernet_udp_transport_write,
    arduino_native_ethernet_udp_transport_read
  ));
    allocator = rcl_get_default_allocator();

    if (!options_initialized) {
        rcl_init_options_t local_init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&local_init_options, allocator));
        RCCHECK(rcl_init_options_set_domain_id(&local_init_options, DOMAIN_ID));
        init_options = local_init_options;  
        options_initialized = true;
    }

    // 1. Core structures must be initialized FIRST
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&teensy_node, "obc_node", "obc", &support));

    #ifdef USING_LED
    if(!led_setup_subscription(&teensy_node, &support, &allocator)){return false;}
    #endif

    #ifdef USING_SERVO
    if(!servo_setup_subscription(&teensy_node, &support, &allocator)){return false;}
    #endif

     #ifdef USING_LORA
    if(!viper_setup_subscription(&teensy_node, &support, &allocator)){return false;}
    #endif

     RCCHECK(rclc_publisher_init_default(
        &imu_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
        "imu"
    ));
     RCCHECK(rclc_publisher_init_default(
        &gps_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), 
        "gps"
    ));
     RCCHECK(rclc_publisher_init_default(
        &tsb_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
        "tsb"
    ));
    RCCHECK(rclc_publisher_init_default(
        &hydrogen_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "hydrogen"
    ));
    RCCHECK(rclc_publisher_init_default(
        &ozone_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "o3"
    ));
    RCCHECK(rclc_publisher_init_default(
        &health_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray), 
        "health"
    ));
#endif
    digitalWrite(LED_PIN, HIGH);
    return true;
}

void obc_setup_imu()
{
#ifdef USING_IMU_ONBOARD
    digitalWrite(IMU_INT1, LOW);
    delay(200);
    while (LSM6DSMR.begin() != LSM6DSR_OK) { delay(100); }
    LSM6DSMR.Set_X_FS(8);
    LSM6DSMR.Set_G_FS(2000);
    LSM6DSMR.Enable_X();
    LSM6DSMR.Enable_G();
#else
    #ifdef USING_IMU_OTHER
    ICM.begin(Wire1, AD0_VAL);
    while (ICM.status != ICM_20948_Stat_Ok) 
    {
        ICM.begin(Wire1, AD0_VAL);
        delay(100);
    }
    #endif
#endif
}

bool obc_setup_hydrogen()
{
    #ifdef USING_SCIENCE_SENSORS
        if (!hydrogen_sensor.begin()) {return false;}
        hydrogen_sensor.setTempCompensation(hydrogen_sensor.OFF);  
        hydrogen_sensor.changeAcquireMode(hydrogen_sensor.INITIATIVE);
    #endif
    return true;
}

bool obc_setup_ozone()
{
    #ifdef USING_SCIENCE_SENSORS
        if (!ozone_sensor.begin(I2C_ADDRESS_OZONE_SENSOR)) {return false;};
        ozone_sensor.setModes(MEASURE_MODE_PASSIVE);
    #endif
    return true;
}

void setup()
{
    Wire1.begin();
    Wire1.setClock(400000);
    Serial6.begin(38400);
    Serial.begin(115200);
    Ethernet.begin(arduino_mac, arduino_ip);
    pinMode(LED_PIN, OUTPUT);
    pinMode(IMU_INT1, OUTPUT);

   obc_setup_health(); 
   obc_setup_imu();
   obc_setup_gps();
    state_UROS = UROS_FOUND;
    #ifdef USING_LORA
    state_lora = LORA_INIT;
    #endif 
    state_TSB = TSB_INIT;
    state_fans = FANS_INIT;
    state_hydrogen = HYDROGEN_INIT;
    state_ozone = OZONE_INIT;

    #ifdef USING_LED
    LED_setup();
    #endif

    obc_setup_servo_hardware();
}

void Uros_SM(){
   switch (state_UROS) {
    case UROS_INIT: {
      ROS_EXECUTE_INTERVAL(500, state_UROS = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? UROS_FOUND : UROS_INIT;);
      break;
    }
    case UROS_FOUND:{
     if (obc_setup_uros()){
      state_UROS = UROS_OK;
     }
     else {
      state_UROS = UROS_INIT;
     }
     if (state_UROS == UROS_INIT) {
      obc_destory_uros_entities();
      };
      break;
    }
    case UROS_OK:{

      ROS_EXECUTE_INTERVAL(200, state_UROS = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? UROS_OK : UROS_ERROR;);
      
      if (state_UROS == UROS_OK) {
        digitalWrite(LED_PIN, HIGH);
        if ((millis() - prev_time2) > 40) {
            prev_time2 = millis();

            rcl_publish(&imu_pub, &imu_msg, NULL);
            rcl_publish(&gps_pub, &gps_msg, NULL);
            rcl_publish(&tsb_pub, &tsb_msg, NULL);
            rcl_publish(&hydrogen_pub, &hydrogen_msg, NULL);
            rcl_publish(&ozone_pub, &ozone_msg, NULL);
            rcl_publish(&health_pub, &health_msg, NULL);
        }
        #ifdef USING_SERVO
          servo_spin_executor();
        #endif

        #ifdef USING_LORA
          rclc_executor_spin_some(&viper_executor, RCL_MS_TO_NS(10));
        #endif

        #ifdef USING_LED
        rclc_executor_spin_some(&led_executor, RCL_MS_TO_NS(10));
        #endif
        }
        
    
    break;
    }

    case UROS_ERROR:{
      obc_destory_uros_entities();
      digitalWrite(LED_PIN, LOW);
      state_UROS = UROS_INIT;
      break;
    }
      
  default:
    break;
}
}

#ifdef USING_LORA

void LORA_SM() {
  switch (state_lora) {

    case LORA_INIT: {
      int16_t state = radio.begin();
      radio.setPacketSentAction(Packet_Sent);
      state_lora = LORA_TRANSMIT;
      break;
    }

    case LORA_TRANSMIT: {
      String viper_message = format_viper_message();
      String payload =   "LAT:" + String(gps_msg.latitude, 6)
                       + ",LON:" + String(gps_msg.longitude, 6)
                       + ",ALT:" + String(gps_msg.altitude, 2)
                       + ",COV:["
                       + String(gps_msg.position_covariance[8], 2)
                       + "," + String(gps_msg.position_covariance[4], 2)
                       + "," + String(gps_msg.position_covariance[0], 2)
                       + "]"
                       + "," + String(gps_msg.status.status)
                       + "\n"
                       + viper_message;

      Transmitted_Flag = false;
      Transmission_State = radio.startTransmit(payload);
      if (Transmission_State != RADIOLIB_ERR_NONE) {
        state_lora = LORA_FINISH;
      } else {
        state_lora = LORA_FLAG;
      }
      break;
    }

    case LORA_FLAG: {
      if (Transmitted_Flag) {
        state_lora = LORA_FINISH;
      }
      break;
    }

    case LORA_FINISH: {
      radio.finishTransmit();
      state_lora = LORA_DELAY;
      break;
    }

    case LORA_DELAY: {
      if (millis() - prev_time_lora > 1000) {
        prev_time_lora = millis();
        state_lora = LORA_TRANSMIT;
      }
      break;
    }

  }
}
#endif

void HYDROGEN_SM(){
  switch(state_hydrogen){
    case HYDROGEN_INIT:
        if (millis() - prev_time_hydrogen > 5000) {
            prev_time_hydrogen = millis();
            if (obc_setup_hydrogen()) {
            state_hydrogen = HYDROGEN_OK;
            }
        }
      break;

    case HYDROGEN_OK:
      if (hydrogen_sensor.dataIsAvailable())
      {
        update_hydrogen_message(AllDataAnalysis.gasconcentration);
      }
      //IDK how to error check for this class
      break; 

    case HYDROGEN_ERROR:
      break;

    default:
      state_hydrogen = HYDROGEN_INIT;
      break;
  }
}

void OZONE_SM(){
  switch(state_ozone){
    case OZONE_INIT:
      if (millis() - prev_time_ozone > 5000) {
            prev_time_ozone = millis();
            if (obc_setup_ozone()) {
            state_ozone = OZONE_OK;
            }
        }
      break;

    case OZONE_OK:
    {
      int16_t ozoneConcentration = ozone_sensor.readOzoneData();
      if (ozoneConcentration >= 0) {
        update_ozone_message(ozoneConcentration);
      }
    }
      break; 

    case OZONE_ERROR:

      break;

    default:
      state_ozone = OZONE_INIT;
      break;
  }
}

void loop()
{
#ifdef USING_IMU_ONBOARD
    updateLSM6DSM(&LSM6DSMR);
#else
    #ifdef USING_IMU_OTHER
    if (ICM.dataReady())
    {
        ICM.getAGMT();
        updateICM_20948(&ICM);
    }
    #endif
#endif

    update_gps();

#ifdef USING_ROS
    Uros_SM();
#endif 

    update_cooling();

#ifdef USING_LORA
    LORA_SM();
#endif

#ifdef USING_SCIENCE_SENSORS
    HYDROGEN_SM();
    OZONE_SM();
#endif
#ifdef USING_LED
  LED_SM();
#endif

update_health(state_UROS, state_fans, state_TSB, state_lora, state_hydrogen, state_ozone);



delay(1);
}
