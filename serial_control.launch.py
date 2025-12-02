#!/usr/bin/env python3
"""
Launch file for serial-based Pupper control.
Starts the serial commander (reading from Pico W) and karel commander (executing commands).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch serial commander and karel realtime commander."""

    # Serial Commander - reads from Pico W and publishes to gpt4_response_topic
    serial_commander = Node(
        package='pupper_llm',
        executable='serial_commander.py',
        name='serial_commander',
        output='both',
        parameters=[
            {'serial_port': '/dev/ttyACM0'},
            {'baud_rate': 115200}
        ]
    )

    # Karel Realtime Commander - subscribes to gpt4_response_topic and controls Pupper
    karel_commander = Node(
        package='pupper_llm',
        executable='karel_realtime_commander.py',
        name='karel_realtime_commander',
        output='both'
    )

    nodes = [
        serial_commander,
        karel_commander,
    ]

    return LaunchDescription(nodes)
