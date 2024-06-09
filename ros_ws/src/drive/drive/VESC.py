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

class Status(Enum):
    STATUS_1 = 9
    STATUS_2 = 14
    STATUS_3 = 15
    STATUS_4 = 16
    STATUS_5 = 27
    STATUS_6 = 28

class VESC_ID(Enum):
    FRONT_LEFT = 88
    FRONT_RIGHT = 87
    BACK_LEFT = 89
    BACK_RIGHT = 86


class VESC:
    """
    VESC class that converts commands into CAN messages

    vesc_id: can id of VESC
    """
    def __init__(self, vesc_id: VESC_ID):
        self.id: VESC_ID = vesc_id
        self.can = CANraw()
        self.scaling = 1
        self.cmd_id: CMD = CMD.CAN_PACKET_SET_RPM
        self.data = 0
    
    def get_can_message(self) -> CANraw:
        self.can.address = self.id.value | (self.cmd_id.value << 8)
        self.can.data = struct.pack(">q", (self.data << 32) * self.scaling)
        self.can.extended = True
        return self.can
    
    def set_duty(self, duty_cycle: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_DUTY
        self.data = duty_cycle
        self.scaling = 100000

    def set_current(self, current: int, brake: bool) -> None:
        self.cmd_id = (CMD.CAN_PACKET_SET_CURRENT_BRAKE if brake else 
                       CMD.CAN_PACKET_SET_CURRENT)
        self.data = current
        self.scaling = 1000
    
    def set_rpm(self, rpm: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_RPM
        rpm_min = 249
        if (rpm < 0): 
            rpm_min = -rpm_min
        self.data = int(rpm * 10 * 3 + rpm_min)
        self.scaling = 1
    
    def set_speed_mps(self, speed: float) -> None:
        d = 0.22
        mps_to_rpm = round((speed / (pi*d)) * 60.0)
        self.set_rpm(mps_to_rpm)
    
    def set_pos(self, deg: int) -> None:
        self.cmd_id = CMD.CAN_PACKET_SET_POS
        self.data = deg
        self.scaling = 1000000
    