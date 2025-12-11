from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("zx200", package_name="zx200_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)

def generate_spawn_controllers_launch(moveit_config):
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ld = LaunchDescription()
    for controller in controller_names:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
    return ld
