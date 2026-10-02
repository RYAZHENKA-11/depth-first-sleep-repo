import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Launch arguments are declared with NO default on purpose. An argument that
# carries a default is always "set", so its value would be pushed as a
# parameter override even when the user never mentioned it — and an empty
# string override (robot_ip:="") beats the node's own default and the value in
# config/go2_webrtc_bridge.yaml. Declaring them without defaults lets
# perform() fail for the ones the user did not pass, and those are then simply
# omitted so the config file / node default wins.
#
# Order matters: the config file is listed first, the command-line overrides
# after it, so a passed argument always beats the config.


def generate_launch_description():
    str_args = (
        "robot_ip",
        "aes_128_key",
        "connection_method",
        "lidar_decoder",
        "imu_source",
    )
    bool_args = (
        "lidar_auto_enable",
        "enable_slam_subscriptions",
        "enable_cmd_vel",
        "lidar_use_payload_frame",
        "publish_full_arrays",
    )

    def build(context, *args, **kwargs):
        lc = LaunchConfiguration

        def given(name):
            """Return the argument's value, or None if it was not passed."""
            try:
                return lc(name).perform(context)
            except Exception:
                return None

        params = {}

        for name in str_args:
            v = given(name)
            if v is not None:
                params[name] = v

        for name in bool_args:
            v = given(name)
            if v is not None:
                # Launch hands every argument over as a string; without an
                # explicit value_type rclpy is handed "false" where the node
                # declared a bool and rejects the override.
                params[name] = ParameterValue(
                    lc(name), value_type=bool
                )

        # Comma-separated on the command line; the node wants a list. An empty
        # value means "exclude nothing" rather than "argument not given".
        excl = given("exclude_raw_fields")
        if excl is not None:
            params["exclude_raw_fields"] = [f.strip() for f in excl.split(",") if f.strip()]

        # ship the package's own config as the base layer, so robot_ip and the
        # recording settings can be edited in one place instead of being
        # retyped on every command line.
        config = None
        try:
            from ament_index_python.packages import get_package_share_directory

            config = os.path.join(
                get_package_share_directory("go2_webrtc_bridge"),
                "config",
                "go2_webrtc_bridge.yaml",
            )
        except Exception:
            config = None

        return [
            Node(
                package="go2_webrtc_bridge",
                executable="bridge_node",
                name="go2_webrtc_bridge",
                output="screen",
                parameters=([config] if config else []) + [params],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(name)
            for name in str_args + bool_args
        ]
        + [
            DeclareLaunchArgument("exclude_raw_fields"),
            OpaqueFunction(function=build),
        ]
    )
