import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("go2_controller")
    params_file = LaunchConfiguration("params_file")
    auto_start = LaunchConfiguration("auto_start")
    dry_run = LaunchConfiguration("dry_run")

    return LaunchDescription([
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(share, "config", "controller.yaml")),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        Node(
            package="go2_controller",
            executable="controller_node",
            name="go2_controller",
            output="screen",
            parameters=[
                params_file,
                {"auto_start": ParameterValue(auto_start, value_type=bool),
                 "dry_run": ParameterValue(dry_run, value_type=bool)},
            ],
        ),
    ])
