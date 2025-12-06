#!/usr/bin/env python3
"""
Test 2: Sensor Commander Test (No Serial Required)
Tests the combined read_data.py in test mode.
Alternates turn_left and turn_right every 10 seconds.

Usage: python3 test_sensor_commander.py
"""

import subprocess
import os

def main():
    print("=" * 50)
    print("TEST 2: Sensor Commander Test")
    print("=" * 50)
    print("This runs read_data.py in test mode.")
    print("Robot will turn left, wait 10s, turn right, repeat.")
    print("Press Ctrl+C to stop.\n")
    
    # Change to project root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(project_root)
    
    print(f"Running from: {project_root}")
    print("Starting sensor commander in test mode...\n")
    
    try:
        subprocess.run(["python3", "read_data.py", "--test"])
    except KeyboardInterrupt:
        print("\nTest stopped by user.")


if __name__ == '__main__':
    main()
