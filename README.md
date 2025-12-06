# Overview
We are using a custom ROS2 publisher and subscriber node to publish sensor readings from a Raspberry Pi Pico W. The Raspberry Pi 4 from Pupper will be subscribed to this node.

# Changes to lab_6_fall_2025:
- Added files:
    - movement_subscriber.py (subscriber node logic)
    - read_data.py (publisher logic; print sensor data to terminal)
    - added movement_subscriber (line 108) and read_data (line 101) as nodes to lab_7.launch.py
    - sense_tension.py (file run by the Raspbery Pi Pico W)
    - run_sensors.sh (script to run launch, read_data.py, and movement_subscriber.py)

NOTE: The logic in movement_subscriber.py was merged into read_data.py to allow for easier integration with ROS2. Used karel_realtime_commander.py as a template.


# ROS2 Configuration
To run testing on local env, ensure ROS2 is installed. Follow offical documentation: https://docs.ros.org/en/foxy/Installation.html






