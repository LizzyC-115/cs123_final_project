"""
Used by Pupper's Raspberry Pi 4 to read the serial outputs from the Raspberry Pi Pico W.

STANDALONE TEST SCRIPT - For testing serial communication only.

For integrated Pupper control, use the ROS2 nodes instead:
  - Run: ros2 launch serial_control.launch.py
  - Or see: SERIAL_COMMANDS.md for full documentation

This script simply reads and prints serial data.
For actual robot control, the data is processed by:
  - pupper_llm/serial_commander.py (ROS2 node)
  - pupper_llm/karel/karel_realtime_commander.py (command executor)
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