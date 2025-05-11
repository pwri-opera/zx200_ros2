import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    
    zx200_description_dir=get_package_share_directory("zx200_description")
    zx200_navigation_dir=get_package_share_directory("zx200_navigation")
    gnss_localizer_ros2 = get_package_share_directory("gnss_localizer_ros2")

    gnss_localizer_ros2_launch_file_path=os.path.join(gnss_localizer_ros2, "launch","gnss_localizer_ros2.py")
    ekf_localization_launch_file_path=os.path.join(zx200_navigation_dir,"launch","ekf_localization.launch.py")
    zx200_navigation_launch_file_path=os.path.join(zx200_navigation_dir,"launch","zx200_navigation.launch.py")

    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_localization_launch_file_path),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zx200_navigation_launch_file_path),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gnss_localizer_ros2_launch_file_path),
        ),
    ])