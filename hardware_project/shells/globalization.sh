#!/bin/bash

#source ros2 in my laptop
cd ~/robot_ws/

# Source ROS 2 setup  you should source both
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash

#to connect lap with raspery
export ROS_DOMAIN_ID=44

#colcon build --symlink-install --packages-select robot_description


#source ros2 in my ws
source ~/robot_ws/install/setup.bash


ros2 service call /reinitialize_global_localization std_srvs/srv/Empty


