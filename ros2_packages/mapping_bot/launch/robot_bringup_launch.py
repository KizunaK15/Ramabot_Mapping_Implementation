import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mapping_bot = get_package_share_directory("mapping_bot")
    pkg_sllidar = get_package_share_directory("sllidar_ros2")

    m5_port_arg = DeclareLaunchArgument(
        "m5_port",
        default_value="/dev/ttyUSB1",
        description="Port serial M5Stack Core Grey (/dev/ttyUSB1 atau /dev/m5stack)"
    )
    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Port serial RPLidar A2M8 (/dev/ttyUSB0 atau /dev/rplidar)"
    )
    invert_yaw_arg = DeclareLaunchArgument(
        "invert_yaw",
        default_value="false",
        description="Set true jika arah yaw serial berlawanan dengan putaran LiDAR"
    )

    xacro_file = os.path.join(pkg_mapping_bot, "urdf", "mapping_bot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_config,
            "use_sim_time": False
        }]
    )

    odom_bridge_node = Node(
        package="mapping_bot",
        executable="mapping_odom_bridge.py",
        name="mapping_odom_bridge",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("m5_port"),
            "baud_rate": 115200,
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "invert_yaw": LaunchConfiguration("invert_yaw"),
            "yaw_scale": 1.0,
            "use_sim_time": False
        }]
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, "launch", "sllidar_a2m8_launch.py")
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("lidar_port"),
            "serial_baudrate": "115200",
            "frame_id": "laser_frame",
            "inverted": "false",
            "angle_compensate": "true",
            "scan_mode": "Sensitivity"
        }.items()
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "send_buffer_limit": 10000000,
            "use_sim_time": False
        }]
    )

    return LaunchDescription([
        m5_port_arg,
        lidar_port_arg,
        invert_yaw_arg,
        robot_state_publisher_node,
        odom_bridge_node,
        sllidar_launch,
        foxglove_bridge_node
    ])
