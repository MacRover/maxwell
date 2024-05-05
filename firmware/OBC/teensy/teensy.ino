#include <micro_ros_arduino.h>
#include <ICM_20948.h>
#include <Wire.h>

#include <cstdint>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

#define AD0_VAL 1
#define MG_TO_MS2 0.0098066

rcl_allocator_t allocator;
rclc_support_t support;

rcl_node_t teensy_node;

ICM_20948_I2C ICM;

rcl_publisher_t pub;
sensor_msgs__msg__Imu imu_msg;

uint8_t arduino_mac[] = { 0x04, 0xE9, 0xE5, 0x13, 0x0E, 0x4B };
IPAddress arduino_ip(192, 168, 1, 177);
IPAddress agent_ip(192, 168, 1, 110);


void setup()
{
    Wire.begin();
    Wire.setClock(400000);

    set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

    allocator = rcl_get_default_allocator();

    rcl_ret_t statusRCL;
    do {
        statusRCL = rclc_support_init(&support, 0, NULL, &allocator);
        ICM.begin(Wire, AD0_VAL);
        delay(500);
    } 
    while (statusRCL != RCL_RET_OK && ICM.status != ICM_20948_Stat_Ok);


    rclc_node_init_default(&teensy_node, "teensy_node", "", &support);

    rclc_publisher_init_default(&pub, &teensy_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
}


void loop()
{
    if (ICM.dataReady())
    {
        ICM.getAGMT();
        imu_msg.linear_acceleration.x = ICM.accX() * MG_TO_MS2;
        imu_msg.linear_acceleration.y = ICM.accY() * MG_TO_MS2;
        imu_msg.linear_acceleration.z = ICM.accZ() * MG_TO_MS2;
    }

    rcl_publish(&pub, &imu_msg, NULL);

    delay(100);
}
