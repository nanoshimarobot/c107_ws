from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

pkg_dir = get_package_share_directory("c107_bringup")

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
            package="picomni_rover_ros", 
            executable="picomni_rover_ros", 
            namespace=""
        ),
        Node(
            package="lidar_wheel_slam_demo",
            executable="lidar_wheel_slam_demo",
            namespace="",
            output="screen",
        )
    ]

    return LaunchDescription(list)