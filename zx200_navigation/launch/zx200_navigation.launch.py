import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


use_autostart = True
use_respawn = True


def generate_launch_description():

    zx200_navigation_dir = get_package_share_directory('zx200_navigation')
    navigation_parameters_yaml_file = os.path.join(zx200_navigation_dir, 'params', 'navigation_parameters.yaml')
    navigation_parameters_sim_yaml_file = os.path.join(zx200_navigation_dir, 'params', 'navigation_parameters_sim.yaml')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='zx200_1')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_navigation_xy_goal_tolerance_arg = DeclareLaunchArgument('navigation_xy_goal_tolerance', default_value='1.0')
    use_navigation_yaw_goal_tolerance_arg = DeclareLaunchArgument('navigation_yaw_goal_tolerance', default_value='0.15')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(zx200_navigation_dir, 'map', 'map.yaml'))
    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_navigation_xy_goal_tolerance = LaunchConfiguration('navigation_xy_goal_tolerance')
    use_navigation_yaw_goal_tolerance = LaunchConfiguration('navigation_yaw_goal_tolerance')

    lifecycle_nodes_localization = ['map_server']

    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    def launch_setup(context, *args, **kwargs):
        use_sim_time_str = use_sim_time.perform(context).strip().lower()
        use_sim_bool = use_sim_time_str in ('true', '1', 'yes', 'y', 'on')
        nav_params_file = navigation_parameters_sim_yaml_file if use_sim_bool else navigation_parameters_yaml_file

        param_substitutions = {
            # map yaml
            'yaml_filename': map_yaml_file,

            # # bt_navigator
            # 'bt_navigator.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
            # 'bt_navigator.ros__parameters.odom_topic': [robot_name, '/odom_pose'],

            # # behavior_server
            # 'behavior_server.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
            # 'behavior_server.ros__parameters.local_frame': [robot_name, '/odom'],

            # controller_server
            # 'controller_server.ros__parameters.odom_topic': [robot_name, '/odom_pose'],
            'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': use_navigation_xy_goal_tolerance,
            'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance': use_navigation_yaw_goal_tolerance,

            # # velocity_smoother
            # 'velocity_smoother.ros__parameters.odom_topic': [robot_name, '/odom_pose'],

            # # costmaps
            # 'local_costmap.local_costmap.ros__parameters.global_frame': [robot_name, '/odom'],
            # 'local_costmap.local_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
            # 'global_costmap.global_costmap.ros__parameters.robot_base_frame': [robot_name, '/base_link'],
        }

        configured_params = RewrittenYaml(
            source_file=nav_params_file,
            root_key=robot_name,
            param_rewrites=param_substitutions,
            convert_types=True
        )

        return [
            GroupAction([
                PushRosNamespace(
                    condition=IfCondition(use_namespace),
                    namespace=robot_name
                ),

                Node(
                    condition=IfCondition('true'),
                    name='nav2_container',
                    package='rclcpp_components',
                    executable='component_container_isolated',
                    parameters=[
                        configured_params,
                        {
                            'autostart': use_autostart,
                        }
                    ],
                    output='screen'
                ),

                #########################
                # Localization packages #
                #########################
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'autostart': use_autostart,
                        'node_names': lifecycle_nodes_localization
                    }]
                ),

                #######################
                # Navigation packages #
                #######################
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],

                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),

                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                ),
                
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'autostart': use_autostart,
                        'node_names': lifecycle_nodes_navigation
                    }]
                ),
            ])
        ]

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg, 
        use_navigation_xy_goal_tolerance_arg,
        use_navigation_yaw_goal_tolerance_arg,

        OpaqueFunction(function=launch_setup),
    ])