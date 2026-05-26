#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot_ns = LaunchConfiguration("robot_ns")

    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    rviz_config = LaunchConfiguration("rviz_config")

    # robot_ns が "zx200" の場合: "zx200/"
    # robot_ns が空の場合: ""
    # frame_prefix = PythonExpression(
    #     [
    #         "'",
    #         robot_ns,
    #         "/' if '",
    #         robot_ns,
    #         "' else ''",
    #     ]
    # )

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("zx200_description"),
            "urdf",
            "zx200.xacro",
        ]
    )

    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("zx200_bringup"),
            "config",
            "vehicle_display.rviz",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            # DeclareLaunchArgument(
            #     "robot_ns",
            #     default_value="",
            #     description="TF frame prefix namespace",
            # ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2",
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="false",
                description=(
                    "Launch joint_state_publisher for standalone visualization. "
                    "Set false when another node publishes JointState."
                ),
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/zx200/joint_states",
                description="JointState topic for robot_state_publisher",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file",
            ),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    robot_description,
                    {
                        "publish_robot_description": True,
                        "use_sim_time": True,

                        # TF frame名に prefix を付ける
                        # 例:
                        #   base_link -> zx200/base_link
                        #   body_link -> zx200/body_link
                        #   boom_link -> zx200/boom_link
                        # "frame_prefix": ParameterValue(
                        #     frame_prefix,
                        #     value_type=str,
                        # ),
                    },
                ],
                remappings=[
                    ("joint_states", joint_states_topic),
                ],
            ),

            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    robot_description,
                    {
                        "use_sim_time": True,
                    },
                ],
                condition=IfCondition(use_joint_state_publisher),
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    {"use_sim_time": True},
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )