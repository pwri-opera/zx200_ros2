import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from threading import Event
import tempfile

robot_name = "zx200"
use_autostart = True
use_sim_time = True
use_respawn = True
use_namespace = True
common_prefix_val = ""
tf_prefix_val = ""

opaque_function_complete_event = Event()

# def wait_for_opaque_function(context):
#     opaque_function_complete_event.wait()
#     return []

def load_params(context, **kwargs):
    global configured_params, zx200_ekf_yaml_file
    zx200_navigation_dir = get_package_share_directory('zx200_navigation')
    navigation_parameters_yaml_file = os.path.join(zx200_navigation_dir, 'params', 'navigation_parameters_sim.yaml')
    zx200_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(zx200_navigation_dir, 'config', 'zx200_ekf.yaml'))

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(zx200_navigation_dir, 'map', 'map.yaml'))
    
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
    opaque_function_complete_event.set()

def generate_nodes(context, *args, **kwargs):
    opaque_function_complete_event.wait()
    zx200_unity_dir = get_package_share_directory("zx200_unity")
    zx200_standby_rviz_file = os.path.join(zx200_unity_dir, "rviz2", "zx200_standby_sim.rviz")
    zx200_description_dir = get_package_share_directory("zx200_description")

    zx200_xacro_file = os.path.join(zx200_description_dir, "urdf", "zx200.xacro")
    doc = xacro.parse(open(zx200_xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    lifecycle_nodes_localization = [
        'map_server'
    ]

    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]   

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['--x', '0', 
                        '--y', '0', 
                        '--z', '0', 
                        '--roll', '0', 
                        '--pitch', '0', 
                        '--yaw', '0', 
                        '--frame-id', 'world',
                        '--child-frame-id', 'map']),
        Node(
            package='zx200_navigation',
            executable='odom_broadcaster',
            name='odom_broadcaster',
            output="screen",
            parameters=[{'odom_topic': 'odom'},
                        {'odom_frame': 'odom'},
                        {'base_link_frame': 'base_link'}]
        ),    
        Node(
            package='zx200_navigation',
            executable='poseStamped2Odometry',
            name='poseStamped2ground_truth_odom',
            output="screen",
            parameters=[{'odom_header_frame': "map",
                            'odom_child_frame': "base_link",
                            'poseStamped_topic_name': "base_link/pose",
                            'odom_topic_name': "tracking/ground_truth",
                            'use_sim_time': True}]
        ),                
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output="screen",
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='zx200_navigation',
            executable='fixed_odom_publisher',
            output="screen",
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package = 'zx200_navigation',
            executable = 'fixed_jointstates_publisher',
            output = 'screen'
        ),     
        Node(
            condition=IfCondition('true'),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': use_autostart},
                                            {'use_sim_time':use_sim_time}],
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
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output="screen",
            remappings=[('odometry/filtered','odometry/global')], 
            parameters=[zx200_ekf_yaml_file,
                                        {
                                        'odom0' : 'fixed_odom',
                                        'odom1' : 'tracking/ground_truth',
                                        }]
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
            remappings=[('cmd_vel', 'cmd_vel_nav')]
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
            remappings= [('cmd_vel', 'tracks/cmd_vel')]
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
                    ('cmd_vel', 'cmd_vel_nav'), 
                        ('cmd_vel_smoothed', 'tracks/cmd_vel')]
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
            arguments=["--display-config", zx200_standby_rviz_file]
        ),
    ]


def generate_launch_description():
    common_prefix = LaunchConfiguration('common_prefix')
    use_rviz = LaunchConfiguration('use_rviz')
    common_prefix_arg = DeclareLaunchArgument('common_prefix',default_value='zx200')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    return LaunchDescription([
        common_prefix_arg,
        use_rviz_arg,
        OpaqueFunction(function=load_params),
        # OpaqueFunction(function=wait_for_opaque_function),
        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name),
            OpaqueFunction(function=generate_nodes),
        ])
    ])
