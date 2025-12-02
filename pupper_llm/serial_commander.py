#!/usr/bin/env python3
"""
Serial Commander for Pupper
Reads commands from Raspberry Pi Pico W via serial and publishes to command topic.
Replaces voice command input with serial input from external controller.
"""

import logging
import serial
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("serial_commander")


class SerialCommanderNode(Node):
    """ROS2 node that reads serial data and publishes commands to Pupper."""

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):
        super().__init__('serial_commander_node')

        # Serial configuration
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None

        # ROS2 Publisher - same topic as realtime_voice uses
        self.response_publisher = self.create_publisher(
            String,
            'gpt4_response_topic',
            10
        )

        # Optional: Also publish raw serial data for debugging
        self.serial_publisher = self.create_publisher(
            String,
            '/serial_data',
            10
        )

        # Timer for reading serial data (10Hz = 100ms)
        self.timer = self.create_timer(0.1, self.read_serial_callback)

        # Initialize serial connection
        self.connect_serial()

        logger.info('Serial Commander initialized')

    def connect_serial(self):
        """Connect to the serial port."""
        max_retries = 5
        retry_delay = 2.0

        for attempt in range(max_retries):
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=1
                )
                logger.info(f"✅ Connected to serial port {self.serial_port} at {self.baud_rate} baud")
                return True
            except serial.SerialException as e:
                logger.warning(f"Attempt {attempt + 1}/{max_retries}: Failed to connect to {self.serial_port}: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    logger.error(f"❌ Failed to connect to serial port after {max_retries} attempts")
                    logger.error(f"Please check that the Pico W is connected to {self.serial_port}")
                    logger.error(f"Run 'ls /dev/ttyACM*' to find available serial ports")
                    return False

    def read_serial_callback(self):
        """Read from serial port and publish commands."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            # Check if data is available
            if self.ser.in_waiting > 0:
                # Read a line from serial
                line = self.ser.readline().decode('utf-8').strip()

                if line:
                    logger.info(f"📨 Received: {line}")

                    # Publish raw serial data for debugging
                    serial_msg = String()
                    serial_msg.data = line
                    self.serial_publisher.publish(serial_msg)

                    # Publish to command topic (same as voice system)
                    command_msg = String()
                    command_msg.data = line
                    self.response_publisher.publish(command_msg)

                    logger.info(f"📤 Published command: {line}")

        except serial.SerialException as e:
            logger.error(f"Serial error: {e}")
            # Try to reconnect
            self.connect_serial()
        except UnicodeDecodeError as e:
            logger.warning(f"Failed to decode serial data: {e}")
        except Exception as e:
            logger.error(f"Unexpected error reading serial: {e}")

    def cleanup(self):
        """Clean up serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)

    # You can override the serial port here if needed
    # e.g., node = SerialCommanderNode(serial_port='/dev/ttyACM1')
    node = SerialCommanderNode()

    try:
        logger.info("🚀 Serial Commander started")
        logger.info("Listening for commands from Raspberry Pi Pico W...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
