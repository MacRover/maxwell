#!/usr/bin/env python3

# follow the link to install python-can. Be sure to pip install python-can[pcan] if using pcan
import can  # https://python-can.readthedocs.io/en/stable/installation.html
import struct
import time
import sys

viper_id = 0x1



## Set to true if you want raw messages instead of decoded messages

print_raw = False


def main():
    with can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500_000) as bus:
        return run_tests(bus)


# Original manual tool, kept here if needed. main() runs the full test sequence
def manual_commands(bus):
    #send_can_id(bus=bus, id=0x1A54, value=0xB0)
    while True:
        try:
            for msg in bus:


                if print_raw:
                    print_raw_message(msg)
                else:
                    decode_message(msg)

        except KeyboardInterrupt:
            i = input('command? ')

            if (i == 'exit'):
                exit()
            elif(i == "set card interval"):
                j = input("card interval value? ")
                send_uint32_value(bus=bus, can_id=0x0E, device_id=viper_id, value = int(j))
            elif(i == "set health"):
                j = input("health value? ")
                send_uint32_value(bus=bus, can_id=0x0B, device_id=viper_id, value = int(j))
            elif(i == "get card interval" or i == "get health"):
                send_uint32_value(bus=bus, can_id=0x0A, device_id=viper_id, value = 0)
                send_uint32_value(bus=bus, can_id=0x0D, device_id=viper_id, value = 0)
            elif(i == "eeprom save"):
                send_uint32_value(bus=bus, can_id=0x07, device_id=viper_id, value = 0)
            # elif(i == "eeprom reload"):
            #     send_uint32_value(bus=bus, can_id=0x12, device_id=viper_id, value = 0)
            elif(i == "disable all cards"):
                send_uint32_value(bus=bus, can_id=0x01, device_id=viper_id, value = 0)
            elif(i == "enable all cards"):
                send_uint32_value(bus=bus, can_id=0x03, device_id=viper_id, value = 0)
            elif(i == "disable card"):
                j = input("card number? ")
                send_card_config_msg(bus=bus, can_id=0x00, device_id=viper_id, card = int(j))
            elif(i == "enable card"):
                j = input("card number? ")
                send_card_config_msg(bus=bus, can_id=0x02, device_id=viper_id, card = int(j))
            elif(i == "set mux value"):
                j = input("card number? ")
                send_card_config_msg(bus=bus, can_id=0x06, device_id=viper_id, card = int(j))
            elif(i == "get card data"):
                j = input("card number? ")
                send_card_config_msg(bus=bus, can_id=0x04, device_id=viper_id, card = int(j))
            elif(i == "get all card data"):
                send_uint32_value(bus=bus, can_id=0x05, device_id=viper_id, value = 0)
            elif(i == "freeze"):
                send_uint32_value(bus=bus, can_id=0x08, device_id=viper_id, value = 0)
            elif(i == "unfreeze"):
                send_uint32_value(bus=bus, can_id=0x09, device_id=viper_id, value = 0)



            elif( i == "estop"):
                send_global_message(bus=bus, can_id=0x31, global_id_arg=0xFF)
            elif( i == "disable"):
                send_global_message(bus=bus, can_id=0x00, global_id_arg=0xFF)
            elif( i == "enable"):
                send_global_message(bus=bus, can_id=0x02, global_id_arg=0)
            elif( i == "ping health"):
                send_global_message(bus=bus, can_id=0x03, global_id_arg=0)

def wait_for_test_reply(bus, event, card=0):
    # Test CAN command 0x7E: [event, card, 4-byte big-endian value]
    expected_id = (5 << 25) | (1 << 18) | (0x7E << 8) | (viper_id << 2)
    end = time.monotonic() + 2
    while time.monotonic() < end:
        msg = bus.recv(timeout=0.05)
        if msg is None or msg.arbitration_id != expected_id or len(msg.data) != 6:
            continue
        if msg.data[0] == event and msg.data[1] == card:
            return struct.unpack(">I", msg.data[2:])[0]
    print("  No matching test report within 2 seconds")
    return None


def test_command(bus, command, card=0, value=None, global_command=False):
    # Remove queued old reports before sending this request
    end = time.monotonic() + 0.1
    while time.monotonic() < end:
        bus.recv(timeout=0.001)
    print("  SEND:", "global" if global_command else "local", hex(command),
          "card", card, "value", value)
    if global_command:
        argument = 0xFF if command in [0x00, 0x31] else 0
        send_global_message(bus, command, argument)
    elif value is None:
        send_card_config_msg(bus, command, viper_id, card)
    else:
        send_uint32_value(bus, command, viper_id, value)
    # Global commands overlap local IDs, so their report events use bit 7
    event = command | 0x80 if global_command else command
    return wait_for_test_reply(bus, event, card)


def check_result(name, actual, expected, mask=0xFFFFFFFF):
    passed = actual is not None and (actual & mask) == expected
    print("PASS:" if passed else "FAIL:", name,
          "- expected", expected, "received", actual, "mask", hex(mask))
    return passed


def run_tests(bus):
    # Snapshot is a test-only command: output bits 0..3, state bit 8,
    # freeze bit 9, selected card bits 10..11
    snapshot = test_command(bus, 0x70)
    health_interval = test_command(bus, 0x0A)
    card_interval = test_command(bus, 0x0D)
    if snapshot is None or health_interval is None or card_interval is None:
        print("FAIL: cannot read baseline. Build/flash VIPER_RTOS with VIPER_TEST defined.")
        return 1
    outputs = snapshot & 15
    state = (snapshot >> 8) & 1
    frozen = (snapshot >> 9) & 1
    selected_card = (snapshot >> 10) & 3
    card = 0  # Card used by mux and individual enable/disable commands
    passed = 0
    failed = 0
    test_queue = [
        "health ping", "health interval", "card interval", "freeze", "mux",
        "card data", "all card data", "unfreeze",
        "global disable", "global enable", "disable card", "enable card",
        "disable all", "enable all", "save EEPROM",
    ]
    print("All command tests will run: card power switching and EEPROM writes included.")
    print("Use an isolated bench bus and an independently powered test PC.")
    print("Command/response checks only. Reports add CAN traffic without waiting.")
    try:
        for name in test_queue:
            print("\nTEST:", name)
            ok = False
            match name:
                case "health ping":
                    ok = check_result(name, test_command(bus, 0x03, global_command=True), state)
                case "health interval":
                    actual = test_command(bus, 0x0B, value=500)
                    ok = check_result("set health interval", actual, 500)
                    actual = test_command(bus, 0x0A)
                    ok = check_result("get health interval", actual, 500) and ok
                case "card interval":
                    actual = test_command(bus, 0x0E, value=500)
                    ok = check_result("set card interval", actual, 500)
                    actual = test_command(bus, 0x0D)
                    ok = check_result("get card interval", actual, 500) and ok
                case "freeze":
                    ok = check_result(name, test_command(bus, 0x08), 1)
                case "mux":
                    ok = check_result(name, test_command(bus, 0x06, card), card)
                case "card data":
                    for test_card in range(4):
                        actual = test_command(bus, 0x04, test_card)
                        result = check_result("card data " + str(test_card), actual, test_card)
                        ok = result if test_card == 0 else result and ok
                case "all card data":
                    ok = check_result(name, test_command(bus, 0x05), 4)
                case "unfreeze":
                    ok = check_result(name, test_command(bus, 0x09), 0)
                case "global disable":
                    ok = check_result(name, test_command(bus, 0x00, global_command=True), 0)
                case "global enable":
                    ok = check_result(name, test_command(bus, 0x02, global_command=True), 1)
                case "disable card":
                    actual = test_command(bus, 0x00, card)
                    ok = check_result(name, actual, 0, 1 << card)
                case "enable card":
                    actual = test_command(bus, 0x02, card)
                    ok = check_result(name, actual, 1 << card, 1 << card)
                case "disable all":
                    ok = check_result(name, test_command(bus, 0x01), 0)
                case "enable all":
                    ok = check_result(name, test_command(bus, 0x03), 15)
                case "save EEPROM":
                    print("  EXPECT: EEPROM driver returned OK (0); persistence needs a reboot check")
                    ok = check_result(name, test_command(bus, 0x07), 0)
            if ok:
                passed += 1
            else:
                failed += 1
    finally:
        print("\nRESTORE original runtime settings")
        cleanup_ok = check_result("health interval", test_command(bus, 0x0B, value=health_interval), health_interval)
        cleanup_ok = check_result("card interval", test_command(bus, 0x0E, value=card_interval), card_interval) and cleanup_ok
        cleanup_ok = check_result("selected card", test_command(bus, 0x06, selected_card), selected_card) and cleanup_ok
        cleanup_ok = check_result("freeze", test_command(bus, 0x08 if frozen else 0x09), frozen) and cleanup_ok
        for test_card in range(4):
            bit = outputs & (1 << test_card)
            actual = test_command(bus, 0x02 if bit else 0x00, test_card)
            cleanup_ok = check_result("output " + str(test_card), actual, bit, 1 << test_card) and cleanup_ok
        actual = test_command(bus, 0x02 if state else 0x00, global_command=True)
        cleanup_ok = check_result("state", actual, state) and cleanup_ok
        cleanup_ok = check_result("save restored parameters", test_command(bus, 0x07), 0) and cleanup_ok
        if not cleanup_ok:
            failed += 1
    print("\nSending E-stop last. No test response or automatic PASS is expected.")
    send_global_message(bus, 0x31, 0xFF)
    print("Verify the physical stop separately. Reset VIPER afterward.")
    print("\nCOMMAND CHECKS:", passed, "passed,", failed, "failed")
    print("E-stop was sent but is not verified by these command/response checks.")
    return 1 if failed else 0



def send_global_message(bus: can.BusABC, can_id: int, global_id_arg: int):
    new_msg = can.Message(
        arbitration_id=(((can_id & 0xFF) << 8) | (global_id_arg & 0xFF)),
        data=[],
        is_extended_id=True,
    )
    bus.send(msg=new_msg)



def send_card_config_msg(bus: can.BusABC, can_id: int, device_id: int, card: int):
    new_msg = can.Message(
        arbitration_id=__can_message_id(device_id, can_id, card_id=card),
        data=[],
        is_extended_id=True,
    )
    arr = bytearray(new_msg.data)
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


def __can_message_id(device_id, command_id, card_id = 0):
    # print((5 << 25) | (command_id << 8) | (device_id << 2) | card_id)
    return (5 << 25) | (command_id << 8) | (device_id << 2) | card_id

def print_raw_message(msg):
    if len(msg.data) == 8:
        float_convert = struct.unpack(">d",msg.data)[0]
        print(msg, round(float_convert, 5))
    else:
        print(msg)

def decode_message(msg):

    command_id = (msg.arbitration_id & (0xFF << 8)) >> 8
    card_id = str(msg.arbitration_id & 0b11)

    command = ""

    if (command_id == 0x0D):
        command = "Card Interval"
    elif (command_id == 0x0A):
        command = "Health Interval"
    elif (command_id == 0xF3):
        command = "Temperature Backplane"
    elif (command_id == 0xF4):
        command = "Temperature Card A"
    elif (command_id == 0xF1):
        command = "Input Voltage"  
    elif (command_id == 0xF5):
        command = "Temperature"
    elif (command_id == 0xF6):
        command = "Input Current"
    elif (command_id == 0xF8):
        command = "Power A"
    elif (command_id == 0xF9):
        command = "Current A"
    elif (command_id == 0xFA):
        command = "Voltage A"
    elif (command_id == 0xFC):
        command = "Power B"
    elif (command_id == 0xFD):
        command = "Current B"
    elif (command_id == 0xFE):
        command = "Voltage B"
    elif (command_id == 0xFF):
        command = "Health"
        card_id = "x"
    

    if len(command) == 0:
        #print_raw_message(msg)
        return
    
    value = msg.data

    if len(msg.data) == 8:
        value = struct.unpack(">d",msg.data)[0]
   
    
    print("Card:\t", card_id, "\tCommand:\t", command, "\tValue:\t", value)

    return



if __name__ == "__main__":
    sys.exit(main())
