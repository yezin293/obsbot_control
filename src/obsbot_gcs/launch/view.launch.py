"""Camera picture only -- for a screen an audience looks at.

Exactly the same stack as gcs.launch.py: same driver, same joystick, same
click-to-point, presets, mouse wheel and keyboard. The only difference is the
presentation. Nothing is drawn over the video, there are no side panels, and it
opens fullscreen.

    ros2 launch obsbot_gcs view.launch.py
    ros2 launch obsbot_gcs view.launch.py fullscreen:=false   # maximised window
    ros2 launch obsbot_gcs view.launch.py joystick:=false

F11 toggles fullscreen and Esc leaves it -- with the panels hidden there is no
window chrome to click.
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
    fullscreen = LaunchConfiguration("fullscreen")

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
            description="Start joy_node and joy_to_ptz alongside the view.",
        ),
        DeclareLaunchArgument(
            "fullscreen",
            default_value="true",
            description="Open fullscreen. false opens a maximised window.",
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
            # The config file is loaded first, then these override the
            # presentation only -- every functional parameter still comes from
            # gcs.yaml, so the two launch files cannot drift apart.
            parameters=[gcs_config, {
                "show_overlay": False,
                "show_ladders": True,
                "show_panels": False,
                "fullscreen": fullscreen,
            }],
            output="screen",
        ),
    ])
