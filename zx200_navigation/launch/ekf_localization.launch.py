import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition

robot_name="zx200"
use_namespace=True

def generate_launch_description():

    zx200_description_dir=get_package_share_directory("zx200_description")
    zx200_navigation_dir=get_package_share_directory("zx200_navigation")

    zx200_ekf_yaml_file = LaunchConfiguration('ekf_yaml_file', default=os.path.join(zx200_navigation_dir, 'config', 'zx200_ekf.yaml'))
    xacro_model = os.path.join(zx200_description_dir, "urdf", "zx200.xacro")
    
    doc = xacro.parse(open(xacro_model))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    return LaunchDescription([

        DeclareLaunchArgument('robot_name', default_value='zx200'),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(str(use_namespace)),
                namespace=robot_name
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_map',
                arguments=['--x','21395.178', 
                           '--y','14034.450', 
                           '--z','28.552', 
                           '--roll','0', 
                           '--pitch','0', 
                           '--yaw','0', 
                           '--frame-id', 'world',
                           '--child-frame-id', 'map']
            ),
            Node(
                package='zx200_navigation',
                executable='odom_broadcaster',
                name='odom_broadcaster',
                output="screen",
                parameters=[{'odom_topic': 'odom_pose'},
                            {'odom_frame': 'odom'},
                            {'base_link_frame': 'base_link'}]
            ),            
            Node(
                package='zx200_navigation',
                executable='poseStamped2Odometry',
                name='poseStamped2ground_truth_odom',
                output="screen",
                parameters=[{'odom_header_frame': "world",
                                'odom_child_frame': "base_link",
                                'poseStamped_topic_name': "global_pose",
                                'odom_topic_name': "gnss_odom"}]
            ),                               
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output="screen",
                parameters=[params],
            ), 
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output="screen",
                remappings=[('odometry/filtered','odometry/global')],
                parameters=[zx200_ekf_yaml_file,
                                            {
                                            'use_sim_time' : False, 
                                            'odom0' : 'odom_pose',
                                            'odom1' : 'gnss_odom',
                                            }]
            ),
        ])
    ])
