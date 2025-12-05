"""
ROS2 Subscriber node that receives movement commands from the '/movement_command' topic
and controls the Pupper robot to move left or right.

This node subscribes to commands published by read_data.py (LoadCellPublisher) and
executes the corresponding movement using the KarelPupper API.

Usage: python3 movement_subscriber.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MovementSubscriber(Node):
    """
    ROS2 Subscriber node that receives movement commands and controls Pupper.
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
        
        # Publisher for velocity commands (same as KarelPupper uses)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters (adjust these for your robot)
        self.linear_speed = 0.5  # Speed for left/right movement
        self.move_duration = 0.5  # Duration of each movement in seconds
        
        # Timer for stopping after movement
        self.stop_timer = None
        
        self.get_logger().info('MovementSubscriber initialized - listening on /movement_command')

    def command_callback(self, msg: String):
        """
        Callback for receiving movement commands.
        
        Args:
            msg: String message containing 'left', 'right', or 'stop'
        """
        command = msg.data[1:-1].lower()
        self.get_logger().info(f'Received command: "{command}"')
        
        if command == 'turn_left':
            self.move_left()
        elif command == 'turn_right':
            self.move_right()
        elif command == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: "{command}"')

    def move_left(self):
        """Move the Pupper to the left (strafe left)."""
        self.get_logger().info('Moving LEFT')
        
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = self.linear_speed  # Positive y = left strafe
        move_cmd.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(move_cmd)
        
        # Schedule stop after movement duration
        self.schedule_stop()

    def move_right(self):
        """Move the Pupper to the right (strafe right)."""
        self.get_logger().info('Moving RIGHT')
        
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = -self.linear_speed  # Negative y = right strafe
        move_cmd.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(move_cmd)
        
        # Schedule stop after movement duration
        self.schedule_stop()

    def stop(self):
        """Stop all movement."""
        self.get_logger().info('STOPPING')
        
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(move_cmd)

    def schedule_stop(self):
        """Schedule a stop command after the movement duration."""
        # Cancel any existing stop timer
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        
        # Create a one-shot timer to stop after movement duration
        self.stop_timer = self.create_timer(self.move_duration, self.stop_timer_callback)

    def stop_timer_callback(self):
        """Timer callback to stop movement."""
        self.stop()
        # Cancel the timer after it fires once
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)

    movement_subscriber = MovementSubscriber()

    try:
        rclpy.spin(movement_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        movement_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
