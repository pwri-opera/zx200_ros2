import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


robot_name="zx200"
use_autostart=True
use_sim_time=False
use_respawn=True
use_namespace=True
use_rviz=True

def generate_launch_description():

    zx200_navigation_dir=get_package_share_directory('zx200_navigation')
    zx200_unity_dir = get_package_share_directory("zx200_unity")


    rviz_file = os.path.join(zx200_unity_dir, "rviz2", "zx200_standby.rviz")

    navigation_parameters_yaml_file = os.path.join(zx200_navigation_dir, 'params', 'navigation_parameters.yaml')

    

    map_yaml_file=LaunchConfiguration('map', default=os.path.join(zx200_navigation_dir, 'map', 'map.yaml'))

    lifecycle_nodes_localization = [
                   'map_server']
    
    lifecycle_nodes_navigation = [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother']
    
    param_substitutions = {
        'use_sim_time': str(use_sim_time),
        'yaml_filename': map_yaml_file,
        
         # bt_navigator
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': os.path.join(zx200_navigation_dir, 'params', 'zx200_navigate_to_pose_w_replanning_and_recovery.xml'),
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': os.path.join(zx200_navigation_dir, 'params', 'zx200_navigate_through_poses_w_replanning_and_recovery.xml'),
    }
    
    configured_params = RewrittenYaml(
            source_file=navigation_parameters_yaml_file,
            root_key='zx200',
            param_rewrites=param_substitutions,
            convert_types=True)
    
    return LaunchDescription([

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),

            Node(
                condition=IfCondition('true'),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': use_autostart},
                                               {'use_sim_time':use_sim_time}],
                output='screen'),
            
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
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': use_autostart},
                            {'node_names': lifecycle_nodes_localization}]
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
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                # remappings=[('cmd_vel', 'cmd_vel_nav')]
                remappings=[('cmd_vel', 'cmd_vel_bef_smoothed')]
                ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',                
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                # remappings= [('cmd_vel', 'tracks/cmd_vel')]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}]
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params]
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                remappings=[
                        ('cmd_vel', 'cmd_vel_bef_smoothed'), 
                         ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': use_autostart},
                            {'node_names': lifecycle_nodes_navigation}]
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=["--display-config", rviz_file]
            ),
            
        ])
    ])
