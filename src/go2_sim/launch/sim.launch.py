import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_sim.urdf import build_urdf


def _setup(context):
    share = get_package_share_directory("go2_sim")
    world = LaunchConfiguration("world").perform(context)
    if os.sep not in world:
        world = os.path.join(share, "worlds", f"{world}.yaml")
    mesh_dir = LaunchConfiguration("mesh_dir").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    use_controller = LaunchConfiguration("controller").perform(context).lower() == "true"

    mode = LaunchConfiguration("mode").perform(context)
    actions = [
        Node(package="go2_sim", executable="fake_go2", name="fake_go2", output="screen",
             parameters=[{"world_file": world, "mode": mode}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="log",
             parameters=[{"robot_description": build_urdf(mesh_dir or None)}]),
    ]
    if use_controller:
        controller_launch = os.path.join(get_package_share_directory("go2_controller"),
                                         "launch", "controller.launch.py")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={"auto_start": LaunchConfiguration("auto_start")}.items()))
    if use_rviz:
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "go2_sim.rviz")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="pole"),
        DeclareLaunchArgument("mode", default_value="contract",
                              description="contract: sim publishes /odom /scan TF; "
                                          "bridge: sim publishes /go2/* like the real bridge"),
        DeclareLaunchArgument("mesh_dir", default_value=os.environ.get("GO2_MESH_DIR", "")),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("controller", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
