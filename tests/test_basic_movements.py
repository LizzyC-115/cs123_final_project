#!/usr/bin/env python3
"""
Test 1: Basic Movement Test
Tests each movement command individually with pauses between.
Run this first to verify all movements work.

Usage: python3 test_basic_movements.py
"""

import sys
sys.path.insert(0, '../pupper_llm/karel')
import karel
import time


def main():
    print("=" * 50)
    print("TEST 1: Basic Movement Test")
    print("=" * 50)
    print("This will test each movement one at a time.")
    print("Press Ctrl+C to stop at any time.\n")
    
    pupper = karel.KarelPupper()
    
    movements = [
        ("Move Forward", pupper.move_forward),
        ("Move Backward", pupper.move_backward),
        ("Move Left (strafe)", pupper.move_left),
        ("Move Right (strafe)", pupper.move_right),
        ("Turn Left", pupper.turn_left),
        ("Turn Right", pupper.turn_right),
        ("Wiggle", pupper.wiggle),
        ("Stop", pupper.stop),
    ]
    
    for name, action in movements:
        print(f"\n>>> Testing: {name}")
        input("Press Enter to execute...")
        try:
            action()
            print(f"✅ {name} complete")
        except Exception as e:
            print(f"❌ {name} failed: {e}")
        time.sleep(1)
    
    print("\n" + "=" * 50)
    print("Basic Movement Test Complete!")
    print("=" * 50)


if __name__ == '__main__':
    main()
