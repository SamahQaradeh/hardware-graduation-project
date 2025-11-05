#!/bin/bash

#source ros2 in my laptop
cd ~/robot_ws/

# Source ROS 2 setup  you should source both
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash

#to connect lap with raspery
export ROS_DOMAIN_ID=44


#rm -rf build/cartographer_slam install/cartographer_slam log/
#rm -rf build install log

colcon build --symlink-install --packages-select cartographer_slam

#source ros2 in my ws
source ~/robot_ws/install/setup.bash

#run the map
ros2 launch cartographer_slam cartographer.launch.py

