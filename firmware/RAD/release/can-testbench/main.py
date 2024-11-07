#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time

rad_id = 0x12


def main():
    sleep_time = 3
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500_000) as bus:

        #send_can_id(bus=bus, id=0x1A54, value=0xB0)
        while True:
            try: 
                for msg in bus:
                    if len(msg.data) == 8:
                        float_convert = struct.unpack(">d",msg.data)[0]
                        print(msg, round(float_convert, 5))
                    else:
                        print(msg)
            except KeyboardInterrupt:
                i = input('command? ')
                
                if (i == 'exit'):
                    exit()
                elif(i == "set p"):
                    j = input("p value? ")
                    send_double_value(bus=bus, can_id = 0x05, device_id = rad_id, value=float(j))
                elif(i == "get p"):
                    send_float_value(bus=bus, can_id = 0x06, device_id = rad_id, value=0)
                elif(i == "set i"):
                    j = input("i value? ")
                    send_double_value(bus=bus, can_id = 0x07, device_id = rad_id, value=float(j))
                elif(i == "get i"):
                    send_float_value(bus=bus, can_id = 0x08, device_id = rad_id, value=0)
                elif(i == "set d"):
                    j = input("d value? ")
                    send_double_value(bus=bus, can_id = 0x09, device_id = rad_id, value=float(j))
                elif(i == "get d"):
                    send_float_value(bus=bus, can_id = 0x0A, device_id = rad_id, value=0)
                elif (i == "set target"):
                    j = input("target? ")
                    send_double_value(bus=bus, can_id = 0x01, device_id = rad_id, value=float(j))
                elif(i == "step motor"):
                    j = input("num pulses? ")
                    step_motor(bus=bus, id=rad_id, value=int(j))
                elif(i == "set odom"):
                    j = input("odom value? ")
                    send_uint32_value(bus=bus, can_id=0x0F, device_id=rad_id, value = int(j))
                elif(i == "set health"):
                    j = input("health value? ")
                    send_uint32_value(bus=bus, can_id=0x13, device_id=rad_id, value = int(j))
                elif(i == "get health" or i == "get odom"):
                    send_uint32_value(bus=bus, can_id=0x10, device_id=rad_id, value = 0)
                    send_uint32_value(bus=bus, can_id=0x14, device_id=rad_id, value = 0)
                elif(i == "eeprom save"):
                    send_uint32_value(bus=bus, can_id=0x11, device_id=rad_id, value = 0)
                elif(i == "eeprom reload"):
                    send_uint32_value(bus=bus, can_id=0x12, device_id=rad_id, value = 0)
                elif(i == "start calibration"):
                    send_uint32_value(bus=bus, can_id=0x15, device_id=rad_id, value = 0)
                elif(i == "cancel calibration"):
                    send_uint32_value(bus=bus, can_id=0x16, device_id=rad_id, value = 0)
                elif(i == "set type"):
                    j = input("type? ")
                    send_uint8_value(bus=bus, can_id=0x0B, device_id=rad_id, value = int(j))
                elif(i == "get type"):
                    send_uint32_value(bus=bus, can_id=0x0C, device_id=rad_id, value = 0)
                elif (i == "reboot"):
                    send_uint32_value(bus=bus, can_id=0x53, device_id=rad_id, value = 0)
                elif (i == "assign rad id"):
                    j = input("rad id? ")
                    send_uint8_value(bus=bus, can_id=0x55, device_id=rad_id, value =int(j))
                elif(i == "drvconf tst"):
                    send_uint8_value(bus=bus, can_id=0x17, device_id=rad_id, value =0)
                elif (i == "set stepper speed"):
                    j = input("freq? ")
                    send_uint32_value(bus=bus, can_id=0x03, device_id=rad_id, value = int(j))
                elif (i == "get stepper speed"):
                    send_uint32_value(bus=bus, can_id=0x04, device_id=rad_id, value = 0)
                elif(i == "fix stepper"):
                    send_uint8_value(bus=bus, can_id=0x4B, device_id=rad_id, value =0b1) #INTPOL
                    time.sleep(0.1)
                    send_uint8_value(bus=bus, can_id=0x4F, device_id=rad_id, value =0b1000) #MRES
                    time.sleep(0.1)
                    send_uint8_value(bus=bus, can_id=0x4D, device_id=rad_id, value=0) #DEDGE
                    time.sleep(0.1)
                    send_uint8_value(bus=bus, can_id=0x31, device_id=rad_id, value =5) #CS
                elif( i == "estop"):
                    send_global_message(bus=bus, can_id=0x31, global_id_arg=0xFF)
                elif( i == "disable"):
                    send_global_message(bus=bus, can_id=0x00, global_id_arg=0xFF)
                elif( i == "enable"):
                    send_global_message(bus=bus, can_id=0x02, global_id_arg=0)
                elif( i == "ping health"):
                    send_global_message(bus=bus, can_id=0x03, global_id_arg=0)
                

                
       
        

def send_setpoint(bus: can.BusABC, id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(id, 0x04),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)


def send_global_message(bus: can.BusABC, can_id: int, global_id_arg: int):
    new_msg = can.Message(
        arbitration_id=(((can_id & 0xFF) << 8) | (global_id_arg & 0xFF)),
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


def send_double_value(bus: can.BusABC, can_id: int, device_id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id),
        data=struct.pack(">d", value),
        is_extended_id=True,
    )
    arr = bytearray(new_msg.data)
    # print(arr.hex())
    # print(new_msg.data)
    bus.send(msg=new_msg)

def send_float_value(bus: can.BusABC, can_id: int, device_id: int, value: float):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id),
        data=struct.pack(">f", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)

def send_uint32_value(bus: can.BusABC, can_id: int, device_id: int, value: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id),
        data=struct.pack(">I", value),
        is_extended_id=True,
    )
    bus.send(msg=new_msg)

def send_uint8_value(bus: can.BusABC, can_id: int, device_id: int, value: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id),
        data=struct.pack(">B", value),
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
