from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

pkg_dir = get_package_share_directory("bringup")
glim_dir = get_package_share_directory("glim")

def generate_launch_description():
    list = [
        IncludeLaunchDescription(
            [
                os.path.join(
                    get_package_share_directory("velodyne"),
                    "launch",
                    "velodyne-all-nodes-VLP16-launch.py",
                )
            ],
        ),
        Node(
            package="joy",
            executable="joy_node",
            namespace="",
        ),
        Node(
            package="omni_teleop",
            executable="omni_teleop_node",
            namespace="",
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     namespace="",
        #     arguments=["-d" + os.path.join(pkg_dir, "rviz", "whill_e2e_nav.rviz")],
        # ),
        Node(package="picomni_rover_ros", executable="picomni_rover_ros", namespace=""),
        # Node(
        #     package="glim_ros",
        #     executable="glim_rosnode",
        #     namespace="",
        #     parameters=[
        #         {
        #             "config_path" : os.path.join(glim_dir, "config")
        #         }
        #     ]
        # )
        Node(
            package="lidar_wheel_slam_demo",
            executable="lidar_wheel_slam_demo",
            namespace="",
            output="screen",
        )
    ]

    return LaunchDescription(list)