#!/usr/bin/env python3
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="zx200")
    declare_command_interface_name = DeclareLaunchArgument(
        "command_interface_name", default_value="effort")

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("robot_name", default_value="zx200"))
    ld.add_action(DeclareLaunchArgument("command_interface_name", default_value="effort"))
    ld.add_action(declare_robot_name)
    ld.add_action(declare_command_interface_name)
    ld.add_action(DeclareLaunchArgument(
        "start_com3", default_value="true",
        description="Start the COM3 gateway; false when it is already running separately"))
    ld.add_action(DeclareLaunchArgument(
        "can_port", default_value="can0", description="COM3 CAN interface"))
    ld.add_action(DeclareLaunchArgument(
        "dbc_path", default_value="/usr/local/share/dbc/excavator_com3.dbc"))
    ld.add_action(DeclareLaunchArgument(
        "input_plane", default_value="9", description="CAN survey plane number (1-19)"))
    ld.add_action(DeclareLaunchArgument(
        "output_plane", default_value="6", description="global_pose plane number (1-19)"))
    ld.add_action(DeclareLaunchArgument(
        "pose_timeout", default_value="0.5", description="Maximum pose component age in seconds"))

    # Keep COM3 outside the MoveIt group: its joint_states must not inherit
    # the front_joint_states remap or a second copy of the robot namespace.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("zx200_com3_ros"), "launch", "zx200_com3_ros.launch.py"
        ])),
        condition=IfCondition(LaunchConfiguration("start_com3")),
        launch_arguments={
            "ns": LaunchConfiguration("robot_name"),
            "can_port": LaunchConfiguration("can_port"),
            "dbc_path": LaunchConfiguration("dbc_path"),
            "input_plane": LaunchConfiguration("input_plane"),
            "output_plane": LaunchConfiguration("output_plane"),
            "pose_timeout": LaunchConfiguration("pose_timeout"),
        }.items(),
    ))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


def launch_setup(context, *args, **kwargs):

    robot_name_str = LaunchConfiguration("robot_name").perform(context)
    command_interface_name_str = LaunchConfiguration("command_interface_name").perform(context)

    # Generate the MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_name_str,
            package_name=f"{robot_name_str}_moveit_config"
        )
        .robot_description(
            file_path=f"config/{robot_name_str}_{command_interface_name_str}.urdf.xacro"
        )
        .to_moveit_configs()
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(robot_name_str),
                *generate_demo_launch_switch_command_interface(
                    moveit_config, command_interface_name_str
                )
            ]
        )
    ]

def generate_demo_launch_switch_command_interface(moveit_config, command_interface):
    """
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    actions = []

    # Boolean Launch Args
    actions.append(DeclareBooleanLaunchArg("db", default_value=False,
                                           description="Start MongoDB if true"))
    actions.append(DeclareBooleanLaunchArg("debug", default_value=False,
                                           description="Enable debug mode"))
    actions.append(DeclareBooleanLaunchArg("use_rviz", default_value=False,
                                           description="Launch RViz if true"))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch# If there are virtual joints, broadcast static tf by including virtual_joints launch
    svj = moveit_config.package_path / "launch" / "static_virtual_joint_tfs.launch.py"
    if svj.exists():
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(svj)))
        )

    # Given the published joint states, publish tf for the robot links
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch" / "rsp.launch.py")
            )
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch" / "move_group.launch.py")
            )
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch" / "moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch" / "warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Start the ros2_control node
    yaml_file = str(
        moveit_config.package_path / "config" /
        f"ros2_{command_interface}_controllers.yaml"
    )
    actions.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, yaml_file],
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch" / "spawn_controllers.launch.py")
            )
        )
    )

    return actions
