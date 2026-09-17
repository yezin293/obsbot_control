"""Everything an operator needs: driver + joystick + GCS window.

    ros2 launch obsbot_gcs gcs.launch.py
    ros2 launch obsbot_gcs gcs.launch.py joystick:=false   # GUI only, no stick
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ptz_share = FindPackageShare("obsbot_ptz")
    gcs_share = FindPackageShare("obsbot_gcs")

    ptz_config = LaunchConfiguration("ptz_config")
    joy_config = LaunchConfiguration("joy_config")
    gcs_config = LaunchConfiguration("gcs_config")
    joystick = LaunchConfiguration("joystick")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ptz_config",
            default_value=PathJoinSubstitution([ptz_share, "config", "ptz.yaml"]),
        ),
        DeclareLaunchArgument(
            "joy_config",
            default_value=PathJoinSubstitution([ptz_share, "config", "joystick.yaml"]),
        ),
        DeclareLaunchArgument(
            "gcs_config",
            default_value=PathJoinSubstitution([gcs_share, "config", "gcs.yaml"]),
        ),
        DeclareLaunchArgument(
            "joystick",
            default_value="true",
            description="Start joy_node and joy_to_ptz alongside the GCS.",
        ),

        Node(
            package="obsbot_ptz",
            executable="ptz_node",
            name="obsbot_ptz",
            namespace="obsbot",
            parameters=[ptz_config],
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="obsbot",
            condition=IfCondition(joystick),
            parameters=[joy_config],
            output="screen",
        ),
        Node(
            package="obsbot_ptz",
            executable="joy_to_ptz",
            name="joy_to_ptz",
            namespace="obsbot",
            condition=IfCondition(joystick),
            parameters=[joy_config],
            output="screen",
        ),
        Node(
            package="obsbot_gcs",
            executable="gcs",
            name="obsbot_gcs",
            namespace="obsbot",
            parameters=[gcs_config],
            output="screen",
        ),
    ])
