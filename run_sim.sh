#!/bin/bash

# 1. Kill existing processes
pkill -f gz_sim
pkill -f move_group
pkill -f rviz2
pkill -f robot_state_publisher
pkill -f spawner
pkill -f ros2
sleep 2

# 2. Rendering Fixes for WSL2
# Using software rendering prevents display-related segmentation faults
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
export QT_OFFSCREEN_PIPELINE_ALWAYS_SKIP=1

# 3. Source Environments
source /opt/ros/humble/setup.bash
source ~/gp_arm_abd/install/setup.bash

# 4. Resource Paths
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix my_robot_description)/share

# 5. Network Settings
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 6. Launch
ros2 daemon stop
ros2 daemon start
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/humble/lib
ros2 launch my_robot_control complete_launch.py