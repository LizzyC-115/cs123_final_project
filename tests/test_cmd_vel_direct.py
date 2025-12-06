#!/usr/bin/env python3
"""
Test 4: Direct cmd_vel Publisher Test
Publishes Twist messages directly to cmd_vel without KarelPupper.
Use this to verify the neural_controller is receiving commands.

Usage: python3 test_cmd_vel_direct.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DirectPublisher(Node):
    def __init__(self):
        super().__init__('direct_cmd_vel_test')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Direct cmd_vel publisher ready')

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=2.0):
        """Publish a Twist message for the specified duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        self.get_logger().info(f'Publishing: x={linear_x}, y={linear_y}, z={angular_z} for {duration}s')
        
        start = time.time()
        while time.time() - start < duration:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        
        # Stop
        self.publisher.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Stopped')


def main():
    print("=" * 50)
    print("TEST 4: Direct cmd_vel Publisher Test")
    print("=" * 50)
    print("This bypasses KarelPupper and publishes directly to cmd_vel.")
    print("If this works but KarelPupper doesn't, the issue is in karel.py.\n")
    
    rclpy.init()
    node = DirectPublisher()
    
    tests = [
        ("Forward", 1.0, 0.0, 0.0),
        ("Backward", -1.0, 0.0, 0.0),
        ("Strafe Left", 0.0, 1.0, 0.0),
        ("Strafe Right", 0.0, -1.0, 0.0),
        ("Turn Left", 0.0, 0.0, 1.0),
        ("Turn Right", 0.0, 0.0, -1.0),
    ]
    
    for name, x, y, z in tests:
        print(f"\n>>> Testing: {name}")
        input("Press Enter to execute...")
        try:
            node.publish_twist(x, y, z, duration=2.0)
            print(f"✅ {name} complete")
        except Exception as e:
            print(f"❌ {name} failed: {e}")
        time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()
    
    print("\n" + "=" * 50)
    print("Direct cmd_vel Test Complete!")
    print("=" * 50)


if __name__ == '__main__':
    main()
