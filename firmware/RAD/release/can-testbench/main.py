#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct


def main():
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1_000_000) as bus:
        for msg in bus:
            angle = int.from_bytes(msg.data[0:2], byteorder="little")
            angle_float = struct.unpack("f", msg.data[2:6])[0]
            # print(msg.data[0:2])
            # print(msg.data)
            # print(angle)
            # print(angle * (360/16_383))
            print(angle_float)


if __name__ == "__main__":
    main()
