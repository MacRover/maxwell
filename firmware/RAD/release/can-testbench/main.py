#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time


def main():
    sleep_time = 3
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500_000) as bus:

        #send_can_id(bus=bus, id=0x1A54, value=0xB0)

        send_p_value(bus=bus, id=0x14, value=0.04)
        time.sleep(1)
        get_p_value(bus=bus, id=0x14, value=0.04)
        step_motor(bus=bus, id=0x14, value=-50)
        for msg in bus:
            print(msg)
            #get_p_value(bus=bus, id=0x99, value=0.04)
            

        
        
                


        # send_p_value(bus=bus, id=0x29, value=0.06)
        # send_i_value(bus=bus, id=0x29, value=0.0001)
        # send_d_value(bus=bus, id=0x29, value=0.0)
        
        # print("setpoint 1")
        # send_setpoint(bus=bus, id=0x29, value=145.8)
        # time.sleep(sleep_time)
        # print("setpoint 2")
        # send_setpoint(bus=bus, id=0x29, value=0.0)
        # time.sleep(sleep_time)
        # print("setpoint 3")
        # send_setpoint(bus=bus, id=0x29, value=970.2)
        # time.sleep(sleep_time)

    #     # while True:
    #     #     # new_msg = can.Message(
    #     #     #     arbitration_id=0x0401,
    #     #     #     data=[0x43, 0x01, 0x99, 0x9A],
    #     #     #     is_extended_id=True,
    #     #     # )
    #     #     new_msg = can.Message(
    #     #         arbitration_id=0x0001,
    #     #         data=[0, 25, 0, 1, 3, 1, 4, 1],
    #     #         is_extended_id=True,
    #     #     )
    #     #     bus.send(msg=new_msg)
    #     #     time.sleep(0.2)

    #     for msg in bus:
    #         if ((msg.arbitration_id & 0xFF00) >> 8) == 9:
    #             angle_float = struct.unpack(">f", msg.data[1:5])[0]
    #             # ls_state = msg.data[0]
    #             # ub_state = msg.data[0]
    #             print(f"{angle_float=}")
    #             if angle_float > 1849 and angle_float < 1851:
    #                 # dummy = 256.0
    #                 # data_dummy = struct.pack(">f", dummy)

    #                 # # 0x4380_0000

    #                 # new_msg = can.Message(
    #                 #     arbitration_id=0x0100,
    #                 #     data=[0x43, 0x80, 0x00, 0x00],
    #                 #     is_extended_id=True,
    #                 # )
    #                 # new_msg = can.Message(
    #                 #     arbitration_id=0x0001,
    #                 #     data=[0, 25, 0, 1, 3, 1, 4, 1],
    #                 #     is_extended_id=True,
    #                 # )
    #                 new_msg = can.Message(
    #                     arbitration_id=0x0401,
    #                     data=[0x43, 0x01, 0x99, 0x9A],
    #                     is_extended_id=True,
    #                 )
    #                 bus.send(msg=new_msg)
    #         elif ((msg.arbitration_id & 0xFF00) >> 8) == 14:
    #             kp = struct.unpack(">f", msg.data[0:4])[0]
    #             ki = struct.unpack(">f", msg.data[4:8])[0]
    #             print(f"{kp=} {ki=}")
    #         elif ((msg.arbitration_id & 0xFF00) >> 8) == 15:
    #             kd = struct.unpack(">f", msg.data[0:4])[0]
    #             print(f"{kd=}")


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
