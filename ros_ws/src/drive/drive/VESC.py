import struct
from math import pi
from enum import Enum
from custom_interfaces.msg import CANraw

class CMD(Enum):
    CAN_PACKET_SET_DUTY = 0
    CAN_PACKET_SET_CURRENT = 1
    CAN_PACKET_SET_CURRENT_BRAKE = 2
    CAN_PACKET_SET_RPM = 3
    CAN_PACKET_SET_POS = 4


class VESC:
    """
    VESC class that converts commands into CAN messages

    vesc_id: can id of VESC
    """
    def __init__(self, vesc_id: int):
        self.id = vesc_id
        self.can = CANraw()
        self.scaling = 1
        self.cmd_id = CMD.CAN_PACKET_SET_RPM.value
        self.data = 0
    
    def get_can_message(self) -> CANraw:
        self.can.address = self.id | (self.cmd_id << 8)
        self.can.data = struct.pack(">q", (self.data << 32) * self.scaling)
        self.can.extended = True
        return self.can
    
    def set_duty(self, duty_cycle: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_DUTY.value
        self.data = duty_cycle
        self.scaling = 100000

    def set_current(self, current: int, brake: bool) -> None:
        self.cmd_id = (CMD.CAN_PACKET_SET_CURRENT_BRAKE.value if brake else 
                       CMD.CAN_PACKET_SET_CURRENT.value)
        self.data = current
        self.scaling = 1000
    
    def set_rpm(self, rpm: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_RPM.value
        self.data = rpm
        self.scaling = 1
    
    def set_speed_mps(self, speed: float) -> None:
        d = 0.22
        mps_to_rpm = round((speed / (pi*d)) * 60.0)
        self.set_rpm(mps_to_rpm)
    
    def set_pos(self, deg: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_POS.value
        self.data = deg
        self.scaling = 1000000
    