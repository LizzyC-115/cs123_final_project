"""
ROS2 Subscriber node that receives movement commands from the '/movement_command' topic
and controls the Pupper robot to move left or right.

This node subscribes to commands published by read_data.py (LoadCellPublisher) and
executes the corresponding movement using the KarelPupper API.

Usage: python3 movement_subscriber.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import asyncio
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("movement_subscriber")


class MovementSubscriber(Node):
    """
    ROS2 Subscriber node that receives movement commands and controls Pupper.
    Publishes Twist messages directly to cmd_vel (no KarelPupper to avoid node conflicts).
    """

    def __init__(self):
        super().__init__('movement_subscriber')
        
        # Subscriber for movement commands from read_data.py
        self.subscription = self.create_subscription(
            String,
            '/movement_command',
            self.command_callback,
            10
        )
        
        # Publisher for velocity commands - publish directly to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.move_duration = 2.0  # seconds (same as KarelPupper)
        
        # Command queue with timestamps (async pattern from karel_realtime_commander)
        self.command_queue = asyncio.Queue()
        self.command_timeout = 20.0  # Discard commands older than 20 seconds
        
        logger.info('MovementSubscriber initialized - listening on /movement_command')

    def command_callback(self, msg: String):
        """
        Callback for receiving movement commands.
        Queues commands for async processing.
        """
        command = msg.data[1:-1].lower().strip()
        logger.info(f'Received command: "{command}"')
        
        if command:
            # Queue command with timestamp for async processing
            current_time = time.time()
            command_with_time = (command, current_time)
            asyncio.create_task(self.command_queue.put(command_with_time))

    async def execute_command(self, command: str) -> bool:
        """Execute a single robot command by publishing Twist messages directly."""
        try:
            logger.info(f"⚙️  Executing: {command}")
            
            move_cmd = Twist()
            
            if command in ['left', 'move_left', 'strafe_left']:
                move_cmd.linear.y = self.linear_speed  # Positive y = strafe left
                await self.publish_movement(move_cmd)
            elif command in ['right', 'move_right', 'strafe_right']:
                move_cmd.linear.y = -self.linear_speed  # Negative y = strafe right
                await self.publish_movement(move_cmd)
            elif command in ['stop', 's']:
                await self.stop()
            elif command in ['forward', 'move_forward', 'move']:
                move_cmd.linear.x = self.linear_speed
                await self.publish_movement(move_cmd)
            elif command in ['backward', 'move_backward', 'back']:
                move_cmd.linear.x = -self.linear_speed
                await self.publish_movement(move_cmd)
            elif command in ['turn_left']:
                move_cmd.angular.z = self.angular_speed
                await self.publish_movement(move_cmd)
            elif command in ['turn_right']:
                move_cmd.angular.z = -self.angular_speed
                await self.publish_movement(move_cmd)
            elif command == '[]' or command == '':
                await asyncio.sleep(0.1)
            else:
                logger.warning(f"⚠️  Unknown command: {command}")
                return False
            
            logger.info(f"✅ Done: {command}")
            return True
            
        except Exception as e:
            logger.error(f"❌ Error executing {command}: {e}")
            return False

    async def publish_movement(self, move_cmd: Twist):
        """Publish movement command for the duration, then stop."""
        start_time = time.time()
        while time.time() - start_time < self.move_duration:
            self.cmd_vel_publisher.publish(move_cmd)
            await asyncio.sleep(0.01)  # Publish at ~100Hz
        await self.stop()

    async def stop(self):
        """Stop all movement."""
        logger.info("Stopping...")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(move_cmd)
        await asyncio.sleep(0.1)

    async def command_processor_loop(self):
        """Process commands from the queue with timeout checking."""
        logger.info("🔄 Command processor started")
        
        while rclpy.ok():
            try:
                # Get next command with timestamp (wait up to 0.1s)
                command_data = await asyncio.wait_for(
                    self.command_queue.get(),
                    timeout=0.1
                )
                
                # Unpack command and timestamp
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
        """Main run loop."""
        await self.command_processor_loop()


async def spin_ros_async(executor):
    """Spin ROS2 executor in async-friendly way."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async(args=None):
    """Async main function."""
    rclpy.init(args=args)
    
    node = MovementSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        logger.info("🚀 Movement Subscriber started")
        logger.info("Ready to receive commands on /movement_command")
        
        # Create tasks for ROS spinning and command processing
        ros_task = asyncio.create_task(spin_ros_async(executor))
        command_task = asyncio.create_task(node.run())
        
        await asyncio.gather(ros_task, command_task)
        
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
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
