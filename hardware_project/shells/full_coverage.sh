#!/bin/bash
# Navigate to the workspace
cd ~/robot_ws/

# Source ROS 2 setup  you should source both
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash


#this to open in laptop and raspery
export ROS_DOMAIN_ID=44

#rm -rf build/ install/ log/
colcon build --symlink-install --packages-select full_coverage

# Source the workspace setup
source ~/robot_ws/install/setup.bash

# Run the ROS 2 launch file
ros2 launch full_coverage nav_client.launch.py



