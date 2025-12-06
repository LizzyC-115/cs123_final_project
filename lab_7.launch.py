from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pupper_v3_description"),
                    "description",
                    "pupper_v3.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Use neural_controller package config (same as lab_6)
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("neural_controller"),
                "launch",
                "config.yaml",
            ]
        ),
        allow_substs=True,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "neural_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        output="both",
        parameters=[{"format": "RGB888", "width": 1400, "height": 1050}],
    )

    # # Get the directory of this launch file to find Python scripts
    # launch_file_dir = os.path.dirname(os.path.abspath(__file__))

    # # Load cell publisher - reads serial data and publishes movement commands
    # load_cell_publisher = ExecuteProcess(
    #     cmd=['python3', os.path.join(launch_file_dir, 'read_data.py')],
    #     name='load_cell_publisher',
    #     output='both',
    # )

    # # Movement subscriber - receives commands and controls Pupper movement
    # movement_subscriber = ExecuteProcess(
    #     cmd=['python3', os.path.join(launch_file_dir, 'movement_subscriber.py')],
    #     name='movement_subscriber',
    #     output='both',
    # )

    nodes = [
        robot_state_publisher,
        imu_sensor_broadcaster_spawner,
        control_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        camera_node,
        # load_cell_publisher,
        # movement_subscriber,
    ]

    return LaunchDescription(nodes)
