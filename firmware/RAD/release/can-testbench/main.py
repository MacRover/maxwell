#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct


def main():
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1_000_000) as bus:
        for msg in bus:
            if ((msg.arbitration_id & 0xFF00) >> 8) == 9:
                angle_float = struct.unpack(">f", msg.data[1:5])[0]
                # ls_state = msg.data[0]
                # ub_state = msg.data[0]
                print(f"{angle_float=}")
            elif ((msg.arbitration_id & 0xFF00) >> 8) == 14:
                kp = struct.unpack(">f", msg.data[0:4])[0]
                ki = struct.unpack(">f", msg.data[4:8])[0]
                print(f"{kp=} {ki=}")
            elif ((msg.arbitration_id & 0xFF00) >> 8) == 15:
                kd = struct.unpack(">f", msg.data[0:4])[0]
                print(f"{kd=}")


if __name__ == "__main__":
    main()
