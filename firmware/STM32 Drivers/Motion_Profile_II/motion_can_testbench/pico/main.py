import sys
import select
from machine import UART, Pin
import time

# Setup UART to talk to the Blue Pill
# TX is GP0, RX is GP1. Match the 115200 baud rate.
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Turn on an LED so we know the code is active
led = Pin("LED", Pin.OUT)
led_external = Pin(15, Pin.OUT)

led.value(0)
led_external.value(0)

for i in range(10):
    led.toggle()
    led_external.toggle()
    time.sleep(0.5)

# Setup a poller to read raw text from the USB port without triggering the REPL
usb_poll = select.poll()
usb_poll.register(sys.stdin, select.POLLIN)

while True:
    # 1. Read from USB (python-can) and pass to UART (Blue Pill)
    if usb_poll.poll(0):
        # Read one character at a time from USB
        char = sys.stdin.read(1)
        if char:
            led.toggle()
            uart.write(char.encode('utf-8'))
            led_external.toggle()

    # 2. Read from UART (Blue Pill) and pass to USB (python-can)
    if uart.any():
        data = uart.read(uart.any())
        led_external.toggle()
        # Write directly to the stdout buffer to avoid formatting issues
        sys.stdout.buffer.write(data)
        # sys.stdout.buffer.flush()
        led.toggle()