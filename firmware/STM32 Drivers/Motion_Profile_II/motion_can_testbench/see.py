import serial
import sys

# Set your COM port here
PORT = 'COM4'
# The baud rate here doesn't strictly matter for the Pico's USB virtual serial port, 
# but matching it to your STM32's baud rate is good practice.
BAUD_RATE = 115200 

try:
    # Open the serial port
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
    print(f"Successfully connected to {PORT}. Listening for CAN frames...\n")
    print("-" * 50)

    while True:
        # Check if there is data waiting in the buffer
        if ser.in_waiting > 0:
            # Read the incoming bytes, decode them to a string, and strip extra newlines
            raw_data = ser.readline()
            try:
                decoded_data = raw_data.decode('utf-8').strip()
                if decoded_data:
                    print(decoded_data)
            except UnicodeDecodeError:
                # Silently ignore any corrupted bytes 
                pass

except serial.SerialException as e:
    print(f"\n[ERROR] Could not open {PORT}.")
    print("Ensure the Pico is plugged in and that Thonny/Arduino IDE are fully closed!")
    print(f"Detailed error: {e}")
except KeyboardInterrupt:
    print("\n\nStopped listening. Exiting script.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()