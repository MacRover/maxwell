#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time

viper_id = 0x1


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
                    send_double_value(bus=bus, can_id = 0x05, device_id = viper_id, value=float(j))
                elif(i == "get p"):
                    send_float_value(bus=bus, can_id = 0x06, device_id = viper_id, value=0)
           
                elif(i == "set odom"):
                    j = input("odom value? ")
                    send_uint32_value(bus=bus, can_id=0x0F, device_id=viper_id, value = int(j))
                elif(i == "set health"):
                    j = input("health value? ")
                    send_uint32_value(bus=bus, can_id=0x13, device_id=viper_id, value = int(j))
                elif(i == "get health" or i == "get odom"):
                    send_uint32_value(bus=bus, can_id=0x10, device_id=viper_id, value = 0)
                    send_uint32_value(bus=bus, can_id=0x14, device_id=viper_id, value = 0)
                elif(i == "eeprom save"):
                    send_uint32_value(bus=bus, can_id=0x11, device_id=viper_id, value = 0)
                elif(i == "eeprom reload"):
                    send_uint32_value(bus=bus, can_id=0x12, device_id=viper_id, value = 0)
                
               
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


def send_uint16_value(bus: can.BusABC, can_id: int, device_id: int, value: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id),
        data=struct.pack(">H", value),
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


def __can_message_id(device_id, card_id, command_id):
    print((5 << 25) | (command_id << 8) | (device_id << 2) | card_id)
    return (5 << 25) | (command_id << 8) | (device_id << 2) | card_id

if __name__ == "__main__":
    main()
