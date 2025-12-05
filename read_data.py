"""
Used by Pupper's Raspberry Pi 4 to read the serial outputs from the Raspberry Pi Pico W.

Reads data from the serial port (load cell/sensor data) and publishes movement commands
to the '/movement_command' topic. A subscriber node will receive these commands and
control the Pupper to move left or right.

Usage: python3 read_data.py
"""

import serial
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class LoadCellPublisher(Node):
    """
    ROS2 Publisher node that reads serial data from Pico W and publishes
    movement commands ('left' or 'right') based on the sensor readings.
    """

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):
        super().__init__('load_cell_publisher')
        
        # Publisher for movement commands
        self.publisher_ = self.create_publisher(String, '/movement_command', 10)
        
        # Initialize serial connection
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.connect_serial()
        
        # Timer to read serial data periodically
        timer_period = 0.1  # 100ms for responsive control
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'LoadCellPublisher initialized on {serial_port}')

    def connect_serial(self):
        """Attempt to connect to the serial port."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to serial port {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.ser = None

    def timer_callback(self):
        # Use for disconnected testing (pivot left and right)
        # while True:
        #     try:
        #         for i in range(10):
        #             msg = String()
        #             msg.data = "[]"
        #             self.publisher_.publish(msg)
        #             self.get_logger().info(f'Published command: "{"[]"}"')
        #             time.sleep(1.0)
        #         msg = String()
        #         msg.data = "[TURN_LEFT]"
        #         self.publisher_.publish(msg)
        #         self.get_logger().info(f'Published command: "{"[TURN_LEFT]"}"')
        #         time.sleep(1.0)
        #         for i in range(10):
        #             msg = String()
        #             msg.data = "[]"
        #             self.publisher_.publish(msg)
        #             self.get_logger().info(f'Published command: "{"[]"}"')
        #             time.sleep(1.0)
        #         msg = String()
        #         msg.data = "[TURN_RIGHT]"
        #         self.publisher_.publish(msg)
        #         self.get_logger().info(f'Published command: "{"[TURN_RIGHT]"}"')
        #         time.sleep(1.0)
        #     except KeyboardInterrupt:
        #         "User stopped test"
        #         return
    
        """Read serial data and publish movement commands."""
        if self.ser is None:
            self.get_logger().warn('Serial not connected, attempting reconnect...')
            self.connect_serial()
            return
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    self.get_logger().info(f'Received: "{line}"')
                    
                    # Parse the data and determine movement command
                    command = self.parse_data(line)
                    
                    if command:
                            msg = String()
                            msg.data = command
                            self.publisher_.publish(msg)
                            self.get_logger().info(f'Published command: "{command}"')
                            time.sleep(1.5)
                        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.ser = None
        except UnicodeDecodeError as e:
            self.get_logger().warn(f'Decode error: {e}')

    def parse_data(self, data: str) -> str:
        data_lower = data.lower().strip()
        
        # Check for explicit direction commands
        if 'left' in data_lower or data_lower == 'l':
            return 'turn_left'
        elif 'right' in data_lower or data_lower == 'r':
            return 'turn_right'
        else:
            return None

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if self.ser:
            self.ser.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create the publisher node
    load_cell_publisher = LoadCellPublisher()

    try:
        rclpy.spin(load_cell_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        load_cell_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()