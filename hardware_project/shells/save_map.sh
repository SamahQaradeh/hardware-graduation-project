#!/bin/bash

#source ros2 in my laptop
cd ~/robot_ws/

# Source ROS 2 setup  you should source both
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/local_setup.bash

#to connect lap with raspery
export ROS_DOMAIN_ID=44

#source ros2 in my ws
source ~/robot_ws/install/setup.bash

cd src/map_server/config


ros2 run nav2_map_server map_saver_cli -f room_area


