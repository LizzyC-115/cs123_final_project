#!/usr/bin/env python3
"""
Test 5: Full System Test
Runs the complete sensor commander with serial connection.
Use this when the Pico W is connected.

Usage: python3 test_full_system.py
"""

import subprocess
import os

def main():
    print("=" * 50)
    print("TEST 5: Full System Test (with Serial)")
    print("=" * 50)
    print("This runs the full sensor commander with serial input.")
    print("Make sure the Pico W is connected to /dev/ttyACM0")
    print("Press Ctrl+C to stop.\n")
    
    # Change to project root
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(project_root)
    
    print(f"Running from: {project_root}")
    print("Starting sensor commander...\n")
    
    try:
        subprocess.run(["python3", "read_data.py"])
    except KeyboardInterrupt:
        print("\nTest stopped by user.")


if __name__ == '__main__':
    main()
