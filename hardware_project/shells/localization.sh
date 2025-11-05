#!/bin/bash
# Navigate to the workspace
cd ~/robot_ws/

# Source ROS 2 setup  you should source both
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash


#this to open in laptop and raspery
export ROS_DOMAIN_ID=44

# Build the workspace (optional if no changes)
#colcon build --symlink-install

colcon build --symlink-install --packages-select map_server
colcon build --symlink-install --packages-select localization_server

# Source the workspace setup
source ~/robot_ws/install/setup.bash

# Run the ROS 2 launch file
ros2 launch localization_server localization.launch.py



#ros2 pkg list | grep sllidar_ros2
#colcon build --packages-select sllidar_ros2
#source install/setup.bash

