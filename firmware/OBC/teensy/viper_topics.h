#ifndef VIPER_TOPICS
#define VIPER_TOPICS

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <custom_interfaces/msg/viper_card_status.h>
#include <custom_interfaces/msg/viper_status.h>
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}


rcl_subscription_t card0_sub, card1_sub, card2_sub, card3_sub, card_health_sub;

rclc_executor_t viper_executor;

custom_interfaces__msg__ViperCardStatus card0_msg, card1_msg, card2_msg, card3_msg;
custom_interfaces__msg__ViperStatus     card_health_msg;

void card0_cb(const void * untyped_msg) {
  const custom_interfaces__msg__ViperCardStatus * incoming = 
    static_cast<const custom_interfaces__msg__ViperCardStatus *>(untyped_msg);

  card0_msg = *incoming;
}
void card1_cb(const void * untyped_msg) {
  const custom_interfaces__msg__ViperCardStatus * incoming = 
    static_cast<const custom_interfaces__msg__ViperCardStatus *>(untyped_msg);

  card1_msg = *incoming;
}
void card2_cb(const void * untyped_msg) {
  const custom_interfaces__msg__ViperCardStatus * incoming = 
    static_cast<const custom_interfaces__msg__ViperCardStatus *>(untyped_msg);

  card2_msg = *incoming;
}
void card3_cb(const void * untyped_msg) {
  const custom_interfaces__msg__ViperCardStatus * incoming = 
    static_cast<const custom_interfaces__msg__ViperCardStatus *>(untyped_msg);

  card3_msg = *incoming;
}
void card_health_cb(const void * untyped_msg) {
  const custom_interfaces__msg__ViperStatus * incoming = 
    static_cast<const custom_interfaces__msg__ViperStatus *>(untyped_msg);

  card_health_msg = *incoming;
}

static size_t formatVoltageA(
  const custom_interfaces__msg__ViperCardStatus * msg,
  char * buf,
  size_t bufSize
) {
  return snprintf(buf, bufSize, "%.2f", msg->output_voltage_a);
}

static size_t formatVoltageAB(
  const custom_interfaces__msg__ViperCardStatus * msg,
  char * buf,
  size_t bufSize
) {
  return snprintf(buf, bufSize, "%.2f,%.2f",
                  msg->output_voltage_a,
                  msg->output_voltage_b);
}
static size_t formatViperCardHealth(
  const custom_interfaces__msg__ViperStatus * msg,
  char * buf,
  size_t bufSize
){

  uint32_t sec  = msg->header.stamp.sec;
  return snprintf(
    buf, bufSize,
    "%u,"    // timestamp, with seconds 
    "%u,%u,%u,%u,%u,%u,"  // six uint8 status fields
    "%.2f",       // input_voltage
    sec,
    msg->eeprom_status,
    msg->mux_status,
    msg->card_0_status,
    msg->card_1_status,
    msg->card_2_status,
    msg->card_3_status,
    msg->input_voltage
  );



}

bool viper_setup_subscription(
    rcl_node_t *node,
    rclc_support_t *support,
    rcl_allocator_t *allocator
){
    RCCHECK(rclc_subscription_init_default(
        &card0_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, ViperCardStatus),
        "/viper_status/card_0"
    ));

 
    RCCHECK(rclc_subscription_init_default(
        &card1_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, ViperCardStatus),
        "/viper_status/card_1"
    ));
    RCCHECK(rclc_subscription_init_default(
        &card2_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, ViperCardStatus),
        "/viper_status/card_2"
    ));
      RCCHECK(rclc_subscription_init_default(
        &card3_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, ViperCardStatus),
        "/viper_status/card_3"
    ));
      RCCHECK(rclc_subscription_init_default(
        &card_health_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, ViperStatus),
        "/viper_status/health"
    ));


    RCCHECK(rclc_executor_init(&viper_executor, &support->context, 5, allocator));


 
    RCCHECK(rclc_executor_add_subscription(
        &viper_executor,
        &card0_sub,
        &card0_msg,
        card0_cb,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &viper_executor,
        &card1_sub,
        &card1_msg,
        card1_cb,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &viper_executor,
        &card2_sub,
        &card2_msg,
        card2_cb,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &viper_executor,
        &card3_sub,
        &card3_msg,
        card3_cb,
        ON_NEW_DATA
    ));

    RCCHECK(rclc_executor_add_subscription(
        &viper_executor,
        &card_health_sub,
        &card_health_msg,
        card_health_cb,
        ON_NEW_DATA
    ));


    return true;


}











#endif 

