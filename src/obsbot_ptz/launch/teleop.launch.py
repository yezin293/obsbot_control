"""Full joystick teleop stack: joy_node -> joy_to_ptz -> ptz_node.

    ros2 launch obsbot_ptz teleop.launch.py
    ros2 launch obsbot_ptz teleop.launch.py camera:=true   # also stream video
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("obsbot_ptz")
    ptz_config = LaunchConfiguration("ptz_config")
    joy_config = LaunchConfiguration("joy_config")
    camera = LaunchConfiguration("camera")
    video_device = LaunchConfiguration("video_device")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ptz_config",
            default_value=PathJoinSubstitution([share, "config", "ptz.yaml"]),
            description="Parameter file for the PTZ driver node.",
        ),
        DeclareLaunchArgument(
            "joy_config",
            default_value=PathJoinSubstitution([share, "config", "joystick.yaml"]),
            description="Parameter file for joy_node and the joystick mapping.",
        ),
        DeclareLaunchArgument(
            "camera",
            default_value="false",
            description="Also start v4l2_camera to publish the video stream. "
                        "Requires the v4l2_camera package.",
        ),
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="Capture node for the video stream (camera:=true only).",
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="obsbot",
            parameters=[joy_config],
            output="screen",
        ),
        Node(
            package="obsbot_ptz",
            executable="joy_to_ptz",
            name="joy_to_ptz",
            namespace="obsbot",
            parameters=[joy_config],
            output="screen",
        ),
        Node(
            package="obsbot_ptz",
            executable="ptz_node",
            name="obsbot_ptz",
            namespace="obsbot",
            # With a camera node capturing, the driver must not stream itself
            # (only one process can); it just checks that someone does.
            parameters=[ptz_config, {"stream": PythonExpression(
                ["'never' if '", camera, "' == 'true' else 'auto'"])}],
            output="screen",
        ),

        # Velocity commands only work while the camera streams. Without this
        # node the driver keeps a small stream of its own; with it, the
        # driver relies on this one.
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            namespace="obsbot",
            condition=IfCondition(camera),
            parameters=[{"video_device": video_device}],
            output="screen",
        ),
    ])
