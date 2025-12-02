"""
Used by Pupper's Raspberry Pi 4 to read the serial outputs from the Raspberry Pi Pico W.
"""

import serial
import time

# Update port if needed: check with `ls /dev/ttyACM*` before running
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

while True:
    # print("Starting to read data...")
    print(ser.readline().decode('utf-8'))
    line = ser.readline().decode('utf-8').strip()
    print(line)
    time.sleep(0.5)