import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("go2_odometry")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share_dir, "config", "odometry.yaml"),
        ),
        Node(
            package="go2_odometry",
            executable="odometry_node",
            name="go2_odometry",
            output="screen",
            parameters=[params_file],
        ),
    ])
