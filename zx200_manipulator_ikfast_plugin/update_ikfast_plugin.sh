#!/bin/bash
# ROS2 MoveIt IKFast Plugin Updater
# This script wraps the original update script for ROS2 compatibility
# Use this instead of update_ikfast_plugin.sh (which gets overwritten by the tool)

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use relative path to the solver
ikfast_output_path="$SCRIPT_DIR/src/zx200_manipulator_ikfast_solver.cpp"

# Get workspace root (parent of src directory)
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Create temporary symlink so the tool can find the package
if [ ! -e "$WORKSPACE_ROOT/zx200_manipulator_ikfast_plugin" ]; then
  ln -s "$SCRIPT_DIR" "$WORKSPACE_ROOT/zx200_manipulator_ikfast_plugin"
  SYMLINK_CREATED=1
fi

cd "$WORKSPACE_ROOT"

ros2 run moveit_kinematics create_ikfast_moveit_plugin.py \
  --search_mode=OPTIMIZE_MAX_JOINT \
  --srdf_filename=zx200.srdf \
  --robot_name_in_srdf=zx200 \
  --moveit_config_pkg=zx200_moveit_config \
  zx200 \
  manipulator \
  zx200_manipulator_ikfast_plugin \
  base_link \
  bucket_end_link \
  "$ikfast_output_path"

# Clean up symlink if we created it
if [ -n "$SYMLINK_CREATED" ] && [ -L "$WORKSPACE_ROOT/zx200_manipulator_ikfast_plugin" ]; then
  rm "$WORKSPACE_ROOT/zx200_manipulator_ikfast_plugin"
fi
