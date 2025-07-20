#ifndef FLASHLIGHT_H
#define FLASHLIGHT_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>


#define FLASHLIGHT_PIN 4
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

rcl_service_t flashlight_service;

std_srvs__srv__SetBool_Request flashlight_request;
std_srvs__srv__SetBool_Response flashlight_response;

rclc_executor_t flashlight_executor;

void set_flashlight(bool state)
{

    if (state)
    {
        digitalWrite(FLASHLIGHT_PIN, HIGH);
    }
    else
    {
        digitalWrite(FLASHLIGHT_PIN, LOW);
    }
    

}

void flashlight_service_callback(const void * req_msg, void * res_msg) {
    // Cast the messages to expected type
    std_srvs__srv__SetBool_Request * request = (std_srvs__srv__SetBool_Request *) req_msg;
    std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *) res_msg;
    
    // Set the flashlight state
    set_flashlight(request->data);
    
    // Prepare response
    response->success = true;
    
    // Set response message
    if (request->data) {
        strcpy(response->message.data, "Flashlight turned ON");
    } else {
        strcpy(response->message.data, "Flashlight turned OFF");
    }
    response->message.size = strlen(response->message.data);
}


bool flashlight_setup_subscription(
    rcl_node_t *node,
    rclc_support_t *support,
    rcl_allocator_t *allocator
) {

    RCCHECK(rclc_service_init_default(
            &flashlight_service,
            node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
            "/flashlight_control"
        ));


    RCCHECK(rclc_executor_init(&flashlight_executor, &support->context, 1, allocator));

    RCCHECK(rclc_executor_add_service(&flashlight_executor, &flashlight_service, &flashlight_request, &flashlight_response, flashlight_service_callback));

    flashlight_response.message.data = (char*) malloc(50 * sizeof(char));
    flashlight_response.message.capacity = 50;

    return true;
}


void flashlight_spin_executor() {
    rclc_executor_spin_some(&flashlight_executor, RCL_MS_TO_NS(100));
}

void flashlight_cleanup(rcl_node_t *node) {
    free(flashlight_response.message.data);
    rclc_executor_fini(&flashlight_executor);
    rcl_service_fini(&flashlight_service, node);
}

#endif // FLASHLIGHT_H