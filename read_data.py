#!/usr/bin/env python3
"""
Sensor Commander - Combined serial reader and robot controller.

Reads data from the serial port (load cell/sensor data from Pico W) and
directly controls the Pupper robot using KarelPupper.

Based on karel_realtime_commander.py async pattern.

Usage: python3 read_data.py
"""

import asyncio
import serial
import time
import logging

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import sys
sys.path.insert(0, 'pupper_llm/karel')
from pupper_llm.karel import karel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("sensor_commander")


class SensorCommanderNode(Node):
    """
    Combined node that reads serial data and controls the robot directly.
    Uses KarelPupper for movement (same pattern as karel_realtime_commander.py).
    """

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200, test_mode=False):
        super().__init__('sensor_commander')
        
        # Initialize serial connection
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.test_mode = test_mode
        
        if not test_mode:
            self.connect_serial()
        
        # Initialize KarelPupper for robot control
        self.pupper = karel.KarelPupper()
        
        # Command queue with timestamps
        self.command_queue = asyncio.Queue()
        self.command_timeout = 20.0  # Discard commands older than 20 seconds
        
        logger.info(f'SensorCommander initialized (test_mode={test_mode})')

    def connect_serial(self):
        """Attempt to connect to the serial port."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logger.info(f'Connected to serial port {self.serial_port}')
        except serial.SerialException as e:
            logger.error(f'Failed to connect to serial port: {e}')
            self.ser = None

    def parse_data(self, data: str) -> str:
        """Parse serial data and return a command string."""
        # return data

        # Use for left, right, backwards, and forwards
        data_lower = data.lower().strip()
        
        # Strip brackets if present (e.g., "[TURN_LEFT]" -> "turn_left")
        if data_lower.startswith('[') and data_lower.endswith(']'):
            data_lower = data_lower[1:-1]
        
        # Map to commands
        if 'turn_left' in data_lower or data_lower == 'l':
            return 'turn_left'
        elif 'turn_right' in data_lower or data_lower == 'r':
            return 'turn_right'
        elif 'move_left' in data_lower:
            return 'move_left'
        elif 'move_right' in data_lower:
            return 'move_right'
        elif 'move_forwards' in data_lower:
            return 'move_forward'
        elif 'move_backwards' in data_lower or 'back' in data_lower:
            return 'move_backward'

        # New commands for two load cells
        elif 'move_forward_left' in data_lower:
            return 'move_diagonal_FL'
        elif 'move_forward_right' in data_lower:
            return 'move_diagonal_FR'
        elif 'move_backward_left' in data_lower:
            return 'move_diagonal_BL'
        elif 'move_backward_right' in data_lower:
            return 'move_diagonal_BR'
        
        elif 'stop' in data_lower:
            return 'stop'
        else:
            return None

    async def read_serial_loop(self):
        """Async loop to read serial data and queue commands."""
        logger.info("🔄 Serial reader started")
        
        while rclpy.ok():
            try:
                if self.test_mode:
                    # Test mode: alternate left/right every 10 seconds
                    await self.run_test_sequence()
                elif self.ser is None:
                    logger.warn('Serial not connected, attempting reconnect...')
                    self.connect_serial()
                    await asyncio.sleep(1.0)
                elif self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        logger.info(f'📡 Received: "{line}"')
                        command = self.parse_data(line)
                        if command:
                            current_time = time.time()
                            await self.command_queue.put((command, current_time))
                            logger.info(f'📋 Queued command: {command}')
                else:
                    await asyncio.sleep(0.05)  # Small delay when no data
                    
            except serial.SerialException as e:
                logger.error(f'Serial read error: {e}')
                self.ser = None
                await asyncio.sleep(1.0)
            except UnicodeDecodeError as e:
                logger.warn(f'Decode error: {e}')
            except Exception as e:
                logger.error(f'Error in serial loop: {e}')
                await asyncio.sleep(0.1)

    async def run_test_sequence(self):
        """Test sequence when no serial connected."""
        # Wait 10 seconds
        for i in range(10):
            logger.info(f'Test mode: waiting... ({10-i}s)')
            await asyncio.sleep(1.0)
        
        # Queue turn_left
        await self.command_queue.put(('turn_left', time.time()))
        logger.info('📋 Test: Queued turn_left')
        
        # Wait 10 seconds
        for i in range(10):
            logger.info(f'Test mode: waiting... ({10-i}s)')
            await asyncio.sleep(1.0)
        
        # Queue turn_right
        await self.command_queue.put(('turn_right', time.time()))
        logger.info('📋 Test: Queued turn_right')

    async def execute_command(self, command: str) -> bool:
        """Execute a single robot command using KarelPupper."""
        try:
            logger.info(f"⚙️  Executing: {command}")

            # coord = [float(i) for i in command.split(',')]
            # self.pupper.move_coordinate(coord)
            
            if command in ['move_forward', 'forward']:
                self.pupper.move_forward()
                await asyncio.sleep(0.5)
            elif command in ['move_backward', 'backward', 'back']:
                self.pupper.move_backward()
                await asyncio.sleep(0.5)
            elif command in ['move_left', 'left', 'strafe_left']:
                self.pupper.move_left()
                await asyncio.sleep(0.5)
            elif command in ['move_right', 'right', 'strafe_right']:
                self.pupper.move_right()
                await asyncio.sleep(0.5)
            elif command == 'turn_left':
                self.pupper.turn_left()
                await asyncio.sleep(0.5)
            
            # New commands or two load cells
            elif command == 'move_diagonal_FL':
                self.pupper.move_diagonal_FL()
                await asyncio.sleep(0.5)
            elif command == 'move_diagonal_FR':
                self.pupper.move_diagonal_FR()
                await asyncio.sleep(0.5)
            elif command == 'move_diagonal_BL':
                self.pupper.move_diagonal_BL()
                await asyncio.sleep(0.5)
            elif command == 'move_diagonal_BR':
                self.pupper.move_diagonal_BR()
                await asyncio.sleep(0.5)


            elif command == 'stop':
                self.pupper.stop()
                await asyncio.sleep(0.1)
            else:
                logger.warning(f"⚠️  Unknown command: {command}")
                return False
            
            logger.info(f"✅ Done: {command}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error executing {command}: {e}")
            return False

    async def command_processor_loop(self):
        """Process commands from the queue."""
        logger.info("🔄 Command processor started")
        
        while rclpy.ok():
            try:
                # Get next command with timestamp
                command_data = await asyncio.wait_for(
                    self.command_queue.get(),
                    timeout=0.1
                )
                
                command, timestamp = command_data
                
                # Check if command is stale
                age = time.time() - timestamp
                if age > self.command_timeout:
                    logger.warning(f"⏰ Discarding stale command '{command}' (age: {age:.1f}s)")
                    continue
                
                # Execute command
                await self.execute_command(command)
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Error in command processor: {e}")
                await asyncio.sleep(0.1)

    async def run(self):
        """Main run loop - runs both serial reader and command processor."""
        serial_task = asyncio.create_task(self.read_serial_loop())
        command_task = asyncio.create_task(self.command_processor_loop())
        await asyncio.gather(serial_task, command_task)

    def destroy_node(self):
        """Clean up."""
        if self.ser:
            self.ser.close()
            logger.info('Serial connection closed')
        super().destroy_node()


async def spin_ros_async(executor):
    """Spin ROS2 executor in async-friendly way."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async(args=None):
    """Async main function."""
    rclpy.init(args=args)
    
    # Check for test mode flag
    test_mode = '--test' in (args or []) or '--test' in sys.argv
    
    node = SensorCommanderNode(test_mode=test_mode)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Add KarelPupper's node to the executor so its publisher works
    executor.add_node(node.pupper.node)
    
    try:
        logger.info("🚀 Sensor Commander started")
        logger.info("📡 Reading from serial port")
        
        # Create tasks
        ros_task = asyncio.create_task(spin_ros_async(executor))
        main_task = asyncio.create_task(node.run())
        
        await asyncio.gather(ros_task, main_task)
        
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def main(args=None):
    """Entry point."""
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Program interrupted")


if __name__ == '__main__':
    main()