import struct
from custom_interfaces.msg import CANraw

CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4

class VESC:
    def __init__(self, vesc_id: int):
        self.id = vesc_id
        self.can = CANraw()
        self.scaling = 1
        self.cmd_id = CAN_PACKET_SET_RPM
        self.data = 0
    
    def get_can_message(self) -> CANraw:
        self.can.address = self.id | (self.cmd_id << 8)
        self.can.data = struct.pack(">i", self.data * self.scaling)
        return self.can
    
    def set_duty(self, duty_cycle: int) -> None:
        self.cmd_id = CAN_PACKET_SET_DUTY
        self.data = duty_cycle
        self.scaling = 100000

    def set_current(self, current: int, brake: bool) -> None:
        self.cmd_id = (CAN_PACKET_SET_CURRENT_BRAKE if brake else 
                       CAN_PACKET_SET_CURRENT)
        self.data = current
        self.scaling = 1000
    
    def set_rpm(self, rpm: int) -> None:
        self.cmd_id = CAN_PACKET_SET_RPM
        self.data = rpm
        self.scaling = 1
    
    def set_pos(self, deg: int) -> None:
        self.cmd_id = CAN_PACKET_SET_POS
        self.data = deg
        self.scaling = 1000000
    