import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    # TODO: Change robot_name from local var into launch argument
    robot_name = "zx200"

    # Create moveit demo launch
    #  Includes
    #  * static_virtual_joint_tfs
    #  * robot_state_publisher
    #  * move_group
    #  * moveit_rviz
    #  * warehouse_db (optional)
    #  * ros2_control_node + controller spawners
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name, package_name=robot_name+"_moveit_config")
        .robot_description(file_path="config/"+robot_name+"_unity.urdf.xacro")
        .to_moveit_configs()
    )
    moveit_demo_launch = generate_demo_launch(moveit_config)

    ld = LaunchDescription()
    ld.add_action(moveit_demo_launch)

    return ld
