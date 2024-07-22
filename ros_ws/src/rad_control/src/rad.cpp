#include "rad_control/rad.hpp"


RAD::RAD(CANraw* can_msg) : l_can_msg(can_msg) { }

RAD::RAD(CANraw* can_msg, uint32_t can_id) : l_can_msg(can_msg)
{
    l_can_msg->address = can_id;
}

void RAD::set_target_angle(double angle)
{
    l_can_data.angle = angle;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_TARGET_ANGLE) << 8);
    _update_can_data();
}

void RAD::set_stepper_speed(uint64_t speed)
{
    l_can_data.speed = speed;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_STEPPER_SPEED) << 8);
    _update_can_data();
}

void RAD::set_p_value(uint32_t P)
{
    l_can_data.P_val = P;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_P_VALUE) << 8);
    _update_can_data();
}

void RAD::set_i_value(uint32_t I)
{
    l_can_data.I_val = I;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_I_VALUE) << 8);
    _update_can_data();
}

void RAD::set_d_value(uint32_t D)
{
    l_can_data.D_val = D;
    l_can_msg->address = (l_can_msg->address & 0xff) | ((uint32_t)(CAN_SET_D_VALUE) << 8);
    _update_can_data();
}

void RAD::_update_can_data()
{
    for (uint8_t i = 0; i < 8; i++)
    {
    #ifdef BIG_ENDIANNESS
        (l_can_msg->data)[i] = l_can_data.arr[i];
    #else
        (l_can_msg->data)[i] = l_can_data.arr[7 - i];
    #endif
    }
}