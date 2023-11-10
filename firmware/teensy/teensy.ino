#include <micro_ros_arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <cstdint>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>

#define USING_IMU
#define USING_GPS

#define LED_PIN 13
#define AD0_VAL 1

#define MG_TO_MS2 0.0098066
#define DEG_TO_RAD 0.01745329


rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t teensy_node;

ICM_20948_I2C ICM;
SFE_UBLOX_GNSS GNSS;

rcl_publisher_t imu_pub;
rcl_publisher_t gps_pub;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;

uint8_t arduino_mac[] = { 0x04, 0xE9, 0xE5, 0x13, 0x0E, 0x4B };
IPAddress arduino_ip(192, 168, 1, 177);
IPAddress agent_ip(192, 168, 1, 110);

unsigned long prev_time1 = 0, prev_time2 = 0;


struct timespec tp;
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);


void updateIMU(ICM_20948_I2C* icm)
{
    imu_msg.linear_acceleration.x = icm->accX() * MG_TO_MS2;
    imu_msg.linear_acceleration.y = icm->accY() * MG_TO_MS2;
    imu_msg.linear_acceleration.z = icm->accZ() * MG_TO_MS2;

    imu_msg.angular_velocity.x = icm->gyrX() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = icm->gyrY() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = icm->gyrZ() * DEG_TO_RAD;
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

void setup()
{
    Wire.begin();
    Wire.setClock(400000);
    Serial1.begin(38400);
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);
    allocator = rcl_get_default_allocator();
    
    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { }

    #ifdef USING_IMU
    ICM.begin(Wire, AD0_VAL);
    while (ICM.status != ICM_20948_Stat_Ok) 
    {
        ICM.begin(Wire, AD0_VAL);
    }
    #endif

    #ifdef USING_GPS
    while (!GNSS.begin(Serial1)) { }
    GNSS.setUART1Output(COM_TYPE_UBX);
    GNSS.setMeasurementRate(33.333);
    GNSS.setNavigationRate(6);
    GNSS.saveConfiguration();
    GNSS.setAutoPVTcallbackPtr(&updatePVTData);
    #endif

    digitalWrite(LED_PIN, HIGH);

    rclc_node_init_default(&teensy_node, "teensy_node", "", &support);

    rclc_publisher_init_default(&imu_pub, &teensy_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
    rclc_publisher_init_default(&gps_pub, &teensy_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "gps");
}


void loop()
{
    #ifdef USING_IMU
    if (ICM.dataReady())
    {
        ICM.getAGMT();
        updateIMU(&ICM);
    }
    #endif
    #ifdef USING_GPS
    GNSS.checkUblox();
    GNSS.checkCallbacks();
    #endif

    if ( (millis() - prev_time2) > 40) 
    {
        prev_time2 = millis();

        rcl_publish(&imu_pub, &imu_msg, NULL);
        rcl_publish(&gps_pub, &gps_msg, NULL);
    }

    delay(1);
}
