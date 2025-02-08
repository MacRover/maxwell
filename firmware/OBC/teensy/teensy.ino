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

#define USING_ROS
#define USING_IMU_ONBOARD
// #define USING_IMU_OTHER
#define USING_GPS
// #define USING_TSB
//#define USING_FANS
#define USING_SERVO


#define DOMAIN_ID 5
#define LED_PIN 13
#define AD0_VAL 1
#define IMU_INT1 23
#define MG_TO_MS2 0.0098066
#define DEG_TO_RAD 0.01745329

#define RCL_RECONNECT(fn){rcl_ret_t rc = (fn); if(rc != RCL_RET_OK){obc_destory_uros_entities();obc_setup_uros();}}


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
IPAddress agent_ip(192, 168, 1, 199);

unsigned long prev_time1 = 0, prev_time2 = 0;


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
    rcl_node_fini(&teensy_node);
    rclc_support_fini(&support);
}

void obc_setup_uros()
{
#ifdef USING_ROS
    set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);
    
    while (rclc_support_init_with_options(
            &support, 0, NULL, 
            &init_options, 
            &allocator) != RCL_RET_OK)
    {
        digitalWrite(LED_PIN, LOW);
    }

    rclc_node_init_default(&teensy_node, "obc_node", "obc", &support);

    rclc_publisher_init_default(
        &imu_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
        "imu"
    );
    rclc_publisher_init_default(
        &gps_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), 
        "gps"
    );
    rclc_publisher_init_default(
        &tsb_pub, 
        &teensy_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), 
        "tsb"
    );
    




#endif
    digitalWrite(LED_PIN, HIGH);
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

void obc_setup_tsb() 
{
#ifdef USING_TSB
    while (!MCP.begin(MCP9601_ADDR, &Wire1)) { delay(100); }
    MCP.setADCresolution(MCP9600_ADCRESOLUTION_18);
    MCP.setThermocoupleType(MCP9600_TYPE_K);
    // setChannel(&tsb1, 0);
#endif
}

void obc_setup_fans()
{
#ifdef USING_FANS
    initializeFan(&fan1, 5);
    initializeFan(&fan2, 6);
    // initializeFan(&fan3, 7);

    enableFanControl(&fan1);
    enableFanControl(&fan2);
    // enableFanControl(&fan3);
#endif
}



void setup()
{
    Wire1.begin();
    Wire1.setClock(400000);
    Serial5.begin(38400);
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(IMU_INT1, OUTPUT);


    obc_setup_imu();
    obc_setup_gps();
    obc_setup_tsb();
    obc_setup_fans();
    obc_setup_uros();
    tsb_init();
    #ifdef USING_SERVO
    servo_setup_subscription(&teensy_node, &support, &allocator);
    #endif
    


}


void loop()
{
tsb_update(&MCP);  

#ifdef USING_SERVO
    servo_spin_executor();
#endif
 

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
    if ( (millis() - prev_time2) > 40) 
    {
        prev_time2 = millis();

        RCL_RECONNECT(rcl_publish(&imu_pub, &imu_msg, NULL));
        RCL_RECONNECT(rcl_publish(&gps_pub, &gps_msg, NULL));
        RCL_RECONNECT(rcl_publish(&tsb_pub, &tsb_msg, NULL)); 
    }
#endif

#ifdef USING_FANS
  setFanRPM(&fan1, MIN_RPM);
  setFanRPM(&fan2, MIN_RPM);
//   setFanRPM(&fan3, MIN_RPM);
#endif


delay(1);
}
