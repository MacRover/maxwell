#include <micro_ros_arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LSM6DSRSensor.h>
#include <Adafruit_MCP9601.h>
#include <Servo.h>

#include <cstdint>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

#include "fans.h"
#include "TSB.h"
#include "servo.h" 
//#define ON_ROVER
#define USING_ROS
#define USING_IMU_ONBOARD
// #define USING_IMU_OTHER
#define USING_GPS
 //#define USING_TSB
//#define USING_FANS
#define USING_SERVO


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

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;



LSM6DSRSensor LSM6DSMR(&Wire1, LSM6DSR_I2C_ADD_H);
ICM_20948_I2C ICM;
SFE_UBLOX_GNSS GNSS;
Adafruit_MCP9601 MCP;

Fan fan1, fan2, fan3;

uint8_t arduino_mac[] = { 0x04, 0xE9, 0xE5, 0x13, 0x0E, 0x4B };
IPAddress arduino_ip(192, 168, 1, 177);
#ifdef ON_ROVER
    IPAddress agent_ip(192, 168, 1, 111);
#else
    IPAddress agent_ip(192, 168, 1, 199);
#endif

unsigned long prev_time1 = 0, prev_time2 = 0, prev_time_fan = 0, prev_time_tsb = 0;
rcl_init_options_t init_options;


struct timespec tp;
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);


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


void updatePVTData(UBX_NAV_PVT_data_t* ubx_nav)
{
    clock_gettime(CLOCK_REALTIME, &tp);
    gps_msg.header.stamp.sec = tp.tv_sec;
    gps_msg.header.stamp.nanosec = tp.tv_nsec;
    gps_msg.header.frame_id.data = "gps_frame";
    gps_msg.header.frame_id.size = 9;
    gps_msg.header.frame_id.capacity = 9;

    gps_msg.latitude = (double)( ubx_nav->lat ) * 0.0000001;
    gps_msg.longitude = (double)( ubx_nav->lon ) * 0.0000001;
    gps_msg.altitude = (double)( ubx_nav->hMSL ) * 0.001;

    double H_m = ubx_nav->hAcc * 0.001;
    double V_m = ubx_nav->vAcc * 0.001;
    gps_msg.position_covariance[0] = H_m*H_m;
    gps_msg.position_covariance[4] = H_m*H_m;
    gps_msg.position_covariance[8] = V_m*V_m;
    gps_msg.position_covariance_type = 
        sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;

    if (ubx_nav->fixType < 2)
        gps_msg.status.status = 
            sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    else
        gps_msg.status.status = 
            sensor_msgs__msg__NavSatStatus__STATUS_FIX;

    gps_msg.status.service = 
        sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
}

void obc_destory_uros_entities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&gps_pub, &teensy_node);
    rcl_publisher_fini(&imu_pub, &teensy_node);
    rcl_publisher_fini(&tsb_pub, &teensy_node);
    rcl_subscription_fini(&servo1_sub, &teensy_node);
    rcl_subscription_fini(&servo2_sub, &teensy_node);
    rcl_subscription_fini(&servo3_sub, &teensy_node);
    rcl_node_fini(&teensy_node);
    rclc_support_fini(&support);
}
static bool options_initialized = false;
bool obc_setup_uros()
{
#ifdef USING_ROS
    allocator = rcl_get_default_allocator();

    if (!options_initialized) {
        rcl_init_options_t local_init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&local_init_options, allocator));
        RCCHECK(rcl_init_options_set_domain_id(&local_init_options, DOMAIN_ID));
        init_options = local_init_options;  
        options_initialized = true;
    }

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&teensy_node, "obc_node", "obc", &support));
    #ifdef USING_SERVO
    if(!servo_setup_subscription(&teensy_node, &support, &allocator)){return false;}
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

void obc_setup_gps()
{
#ifdef USING_GPS
    while (!GNSS.begin(Serial5)) { delay(100); }
    GNSS.setUART1Output(COM_TYPE_UBX);
    GNSS.setMeasurementRate(33.333);
    GNSS.setNavigationRate(6);
    GNSS.saveConfiguration();
    GNSS.setAutoPVTcallbackPtr(&updatePVTData);
#endif
}

bool obc_setup_tsb() 
{
#ifdef USING_TSB
    if (!MCP.begin(MCP9601_ADDR, &Wire1)) { return false; }
    MCP.setADCresolution(MCP9600_ADCRESOLUTION_18);
    MCP.setThermocoupleType(MCP9600_TYPE_K);
    // setChannel(&tsb1, 0);
    tsb_init();  
#endif
return true;
}

bool obc_setup_fans()
{
#ifdef USING_FANS
    initializeFan(&fan1, 5);
    initializeFan(&fan2, 6);
    // initializeFan(&fan3, 7);

    enableFanControl(&fan1);
    enableFanControl(&fan2);
    // enableFanControl(&fan3);
#endif
return true;
}



void setup()
{
    set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);
    Wire1.begin();
    Wire1.setClock(400000);
    Serial5.begin(38400);
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(IMU_INT1, OUTPUT);


    obc_setup_imu();
    obc_setup_gps();
    obc_setup_tsb();
    state_UROS = UROS_FOUND;
    state_TSB = TSB_INIT;
    state_fans = FANS_INIT;
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
        }
        #ifdef USING_SERVO
          servo_spin_executor();
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
void FANS_SM() {
  switch (state_fans) {
    case FANS_INIT:
    if (obc_setup_fans()) {
        state_fans = FANS_OK;
    } 
    else {
        state_fans = FANS_ERROR;
    }
    break;


    case FANS_OK:
        setFanRPM(&fan1, MIN_RPM);
        setFanRPM(&fan2, MIN_RPM);
        // setFanRPM(&fan3, MIN_RPM);
      break;

    case FANS_ERROR:
      // Try to connect to fans every 5 seconds 
      if (millis() - prev_time_fan > 5000){
        prev_time_fan = millis();

        if (obc_setup_fans()){
          state_fans = FANS_OK;
        }
        else{
          state_fans = FANS_ERROR;
        }
      }
      break;
      

    default:
      state_fans = FANS_INIT;
      break;
  }
}

void TSB_SM(){
  switch(state_TSB){
    case TSB_INIT:
      if (obc_setup_tsb()){
        state_TSB = TSB_OK;
      } 
      else {
        state_TSB = TSB_ERROR;
      }
      break;

    case TSB_OK:
      tsb_update(&MCP); 
      break; 

    case TSB_ERROR:
    // Try to connect to TSB every 5 seconds 
      if (millis() - prev_time_tsb > 5000) {
        prev_time_tsb = millis();
        if (obc_setup_tsb()){
          state_TSB = TSB_OK;
        }
        else {
          state_TSB = TSB_ERROR;
        }
      }
      break;

    default:
      state_TSB = TSB_INIT;
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

#ifdef USING_GPS
    GNSS.checkUblox();
    GNSS.checkCallbacks();
#endif

#ifdef USING_ROS
Uros_SM();
#endif 

#ifdef USING_FANS
FANS_SM();
#endif

#ifdef USING_TSB
TSB_SM();
#endif 


delay(1);
}
