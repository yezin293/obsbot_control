"""Driver only: the PTZ node, no joystick.

Useful when something else publishes /obsbot/cmd_ptz (a tracker, a GUI, a test
script).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ptz_config = LaunchConfiguration("ptz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ptz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("obsbot_ptz"), "config", "ptz.yaml"]
            ),
            description="Parameter file for the PTZ driver node.",
        ),
        Node(
            package="obsbot_ptz",
            executable="ptz_node",
            name="obsbot_ptz",
            namespace="obsbot",
            parameters=[ptz_config],
            output="screen",
        ),
    ])
