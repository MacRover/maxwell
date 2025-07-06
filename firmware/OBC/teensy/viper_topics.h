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
custom_interfaces__msg__ViperStatus card_health_msg;

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

String format_viper_message(){
    
    String message;
    // Card 0 
    message += String("C0 OVA: ");
    message += String(card0_msg.output_voltage_a, 2);
    message += ";";
    // Card 1
    mesasge += String(" C1 OVA OVB: ");
    message += String(card1_msg.output_voltage_a, 2);
    message += " ";
    message += String(card1_msg.output_voltage_b, 2);
    message += ";";
    // Card 2
    message += String(" C2 OVA OVB: ");
    message += String(card2_msg.output_voltage_a, 2);
    message += " ";
    message += String(card2_msg.output_voltage_b, 2);
    message += ";"; 
    // card 3
    message += String(" C3 OVA");
    message += String(card3_msg.output_voltage_a, 2);
    message += ";";

    // Card Health
    message += String(" EEPROM: ");
    message+= String(card_health_msg.eeprom_status);
    message+= ",";
    message += String (" MUX: ");
    message+= String(card_health_msg.mux_status);
    message+= ",";

    message += String(" C0: ");
    message+= String(card_health_msg.card_0_status);
    message+= ",";
    message += String(" C1: ");
    message+= String(card_health_msg.card_1_status);
    message+= ",";
    message += String(" C1: ");
    message+= String(card_health_msg.card_2_status);
    message+= ",";
    message += String(" C2: ");
    message+= String(card_health_msg.card_3_status);
    message+= ",";

    message += String(" Input V: ");
    message+= String(card_health_msg.input_voltage, 2);


    return message;

}

void destroy_viper_topics(rcl_node_t *node){
  rcl_subscription_fini(&card0_sub, node);
  rcl_subscription_fini(&card1_sub, node);
  rcl_subscription_fini(&card2_sub, node);
  rcl_subscription_fini(&card3_sub, node);
  rcl_subscription_fini(&card_health_sub, node);
}

#endif 

