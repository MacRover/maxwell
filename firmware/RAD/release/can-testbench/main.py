#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time


def main():
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1_000_000) as bus:
        # while True:
        #     # new_msg = can.Message(
        #     #     arbitration_id=0x0401,
        #     #     data=[0x43, 0x01, 0x99, 0x9A],
        #     #     is_extended_id=True,
        #     # )
        #     new_msg = can.Message(
        #         arbitration_id=0x0001,
        #         data=[0, 25, 0, 1, 3, 1, 4, 1],
        #         is_extended_id=True,
        #     )
        #     bus.send(msg=new_msg)
        #     time.sleep(0.2)
        for msg in bus:
            if ((msg.arbitration_id & 0xFF00) >> 8) == 9:
                angle_float = struct.unpack(">f", msg.data[1:5])[0]
                # ls_state = msg.data[0]
                # ub_state = msg.data[0]
                print(f"{angle_float=}")
                if angle_float > 1849 and angle_float < 1851:
                    # dummy = 256.0
                    # data_dummy = struct.pack(">f", dummy)

                    # # 0x4380_0000

                    # new_msg = can.Message(
                    #     arbitration_id=0x0100,
                    #     data=[0x43, 0x80, 0x00, 0x00],
                    #     is_extended_id=True,
                    # )
                    # new_msg = can.Message(
                    #     arbitration_id=0x0001,
                    #     data=[0, 25, 0, 1, 3, 1, 4, 1],
                    #     is_extended_id=True,
                    # )
                    new_msg = can.Message(
                        arbitration_id=0x0401,
                        data=[0x43, 0x01, 0x99, 0x9A],
                        is_extended_id=True,
                    )
                    bus.send(msg=new_msg)
            elif ((msg.arbitration_id & 0xFF00) >> 8) == 14:
                kp = struct.unpack(">f", msg.data[0:4])[0]
                ki = struct.unpack(">f", msg.data[4:8])[0]
                print(f"{kp=} {ki=}")
            elif ((msg.arbitration_id & 0xFF00) >> 8) == 15:
                kd = struct.unpack(">f", msg.data[0:4])[0]
                print(f"{kd=}")


if __name__ == "__main__":
    main()
