import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='zx200_1')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=robot_name
            ),

            Node(
                package='zx200_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[
                    {'odom_topic': 'fixed_odom', 
                     'odom_frame': '/odom',
                     'base_link_frame': '/base_link',
                     'use_sim_time': use_sim_time,
                    },
                ]
            ),
            Node(
                package='zx200_navigation',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "map",
                                'odom_child_frame': "base_link",
                                'poseStamped_topic_name': "base_link/pose",
                                'odom_topic_name': "global_pose",
                                'use_sim_time': use_sim_time}]
            ),     
            Node(
                package = 'zx200_navigation',
                executable = 'message_converter_odom',
                name = "message_converter_odom",
                output = "screen",
                parameters=[{'input_topic': "odom",
                            'output_topic': 'fixed_odom',
                            'use_sim_time': use_sim_time}],
            ),

            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('odom0', 'fixed_odom'),
                    ('odom1', 'global_pose'),
                ],
                parameters=[{
                    'debug': False,
                    'frequency': 10.0,
                    'transform_time_offset': 0.0,
                    'transform_timeout': 0.0,
                    'print_diagnostics': True,
                    'publish_acceleration': True,
                    'publish_tf': True,
                    'two_d_mode': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_link',
                    'world_frame': 'map',
                    'use_sim_time': use_sim_time,

                    'odom0': 'fixed_odom',
                    'odom0_queue_size': 10,
                    'odom0_config': [
                        True,  True,  False,
                        False, False, True,
                        False, False, False,
                        False, False, False,
                        False, False, False],
                    'odom0_differential': True,

                    'odom1': 'global_pose',
                    'odom1_queue_size': 10,
                    'odom1_config': [
                        True,  True,  True,
                        False, False, True,
                        False, False, False,
                        False, False, False,
                        False, False, False],
                    'odom1_differential': False,
                }]
            ),
        ])
    ])