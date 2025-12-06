#!/usr/bin/env python3
"""
Test 3: Serial Simulation Test
Simulates what the Pico W would send over serial.
Type commands manually to test the full pipeline.

Usage: python3 test_serial_simulation.py
"""

import sys
sys.path.insert(0, '../pupper_llm/karel')
import karel
import time


def parse_command(data: str) -> str:
    """Same parsing logic as read_data.py"""
    data_lower = data.lower().strip()
    
    # Strip brackets if present
    if data_lower.startswith('[') and data_lower.endswith(']'):
        data_lower = data_lower[1:-1]
    
    if 'turn_left' in data_lower or data_lower == 'l':
        return 'turn_left'
    elif 'turn_right' in data_lower or data_lower == 'r':
        return 'turn_right'
    elif 'left' in data_lower:
        return 'move_left'
    elif 'right' in data_lower:
        return 'move_right'
    elif 'forward' in data_lower or data_lower == 'f':
        return 'move_forward'
    elif 'backward' in data_lower or 'back' in data_lower or data_lower == 'b':
        return 'move_backward'
    elif 'stop' in data_lower or data_lower == 's':
        return 'stop'
    elif 'wiggle' in data_lower or data_lower == 'w':
        return 'wiggle'
    else:
        return None


def execute_command(pupper, command: str):
    """Execute a command on the pupper."""
    print(f"⚙️  Executing: {command}")
    
    if command == 'move_forward':
        pupper.move_forward()
    elif command == 'move_backward':
        pupper.move_backward()
    elif command == 'move_left':
        pupper.move_left()
    elif command == 'move_right':
        pupper.move_right()
    elif command == 'turn_left':
        pupper.turn_left()
    elif command == 'turn_right':
        pupper.turn_right()
    elif command == 'stop':
        pupper.stop()
    elif command == 'wiggle':
        pupper.wiggle()
    else:
        print(f"⚠️  Unknown command: {command}")
        return
    
    print(f"✅ Done: {command}")


def main():
    print("=" * 50)
    print("TEST 3: Serial Simulation Test")
    print("=" * 50)
    print("Type commands as if they came from the Pico W.")
    print("Examples: left, right, [TURN_LEFT], forward, l, r, f, b, w")
    print("Type 'quit' or 'q' to exit.\n")
    
    pupper = karel.KarelPupper()
    
    while True:
        try:
            user_input = input("📡 Simulated serial input: ").strip()
            
            if user_input.lower() in ['quit', 'q', 'exit']:
                print("Exiting...")
                break
            
            if not user_input:
                continue
            
            command = parse_command(user_input)
            
            if command:
                execute_command(pupper, command)
            else:
                print(f"❓ Could not parse: '{user_input}'")
            
            time.sleep(0.5)
            
        except KeyboardInterrupt:
            print("\nExiting...")
            break
    
    pupper.stop()
    print("\n" + "=" * 50)
    print("Serial Simulation Test Complete!")
    print("=" * 50)


if __name__ == '__main__':
    main()
