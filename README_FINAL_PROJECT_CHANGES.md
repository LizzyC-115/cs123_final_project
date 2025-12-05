# Overview
We are using a custom ROS2 publisher and subscriber node to publish sensor readings from a Raspberry Pi Pico W. The Raspberry Pi 4 from Pupper will be subscribed to this node.

# Changes to lab_7_fall_2025:
- Added files:
    - movement_subscriber.py (subscriber node logic)
    - read_data.py (publisher logic; print sensor data to terminal)
    - added movement_subscriber (line 108) and read_data (line 101) as nodes to lab_7.launch.py


# ROS2 Configuration
To run testing on local env, ensure ROS2 is installed. Follow offical documentation: https://docs.ros.org/en/foxy/Installation.html






