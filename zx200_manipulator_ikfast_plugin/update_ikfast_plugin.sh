search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=zx200.srdf
robot_name_in_srdf=zx200
moveit_config_pkg=zx200_moveit_config
robot_name=zx200
planning_group_name=manipulator
ikfast_plugin_pkg=zx200_manipulator_ikfast_plugin
base_link_name=base_link
eef_link_name=bucket_end_link
ikfast_output_path=/home/common/ros2_ws/zx200_manipulator_ikfast_plugin/src/zx200_manipulator_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
