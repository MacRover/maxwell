#!/usr/bin/env python3

from can import Message
from can.interface import Bus


def main():
    bus = Bus(interface="socketcan", channel="can0", bitrate=500000)
    message = Message(
        is_extended_id=True, arbitration_id=0xC0FFEE, data=b"hi world"
    )
    bus.send(message)


if __name__ == "__main__":
    main()
