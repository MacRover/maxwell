#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time


def main():
    sleep_time = 3
    with can.ThreadSafeBus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500_000) as bus:

        #send_can_id(bus=bus, id=0x1A54, value=0xB0)
  
        while True:
            i = input("command? ")

            if (i == 'exit'):
                break
            elif(i == "set pid"):
                send_p_value(bus=bus, id=0x14, value=0.04)
            elif(i == "get pid"):
                get_p_value(bus=bus, id=0x14, value=0.04)
            elif(i == "step motor"):
                step_motor(bus=bus, id=0x14, value=50)
                #     get_p_value(bus=bus, id=0x14, value=0.04)

                #     step_motor(bus=bus, id=0x14, value=-50)
                #     continue
       

        
        

def send_setpoint(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x04),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_reset(bus: can.BusABC, id: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x49),
        data=[],
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_p_value(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x05),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)

def get_p_value(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x06),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)

def step_motor(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x51),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_i_value(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x47),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_d_value(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x48),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_can_id(bus: can.BusABC, id: int, value: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0),
        data=[0, 0, 0, 0, 0, 0, 0, value],
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def __can_message_id(device_id, command_id):
    print((2 << 25) | (command_id << 8) | device_id)
    return (2 << 25) | (command_id << 8) | device_id


if __name__ == "__main__":
    main()
