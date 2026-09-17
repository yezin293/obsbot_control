from glob import glob

from setuptools import setup

package_name = "obsbot_ptz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Judy",
    maintainer_email="judy293@koreatech.ac.kr",
    description="ROS 2 driver and joystick teleop for OBSBOT PTZ cameras over V4L2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ptz_node = obsbot_ptz.ptz_node:main",
            "joy_to_ptz = obsbot_ptz.joy_to_ptz_node:main",
            "probe = obsbot_ptz.probe:main",
        ],
    },
)
