from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    configuration_basename = 'cartographer.lua'

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-resolution', '0.05',
                '-publish_period_sec', '1.0'
            ]
        )
        
        ,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/samah/robot_ws/src/map_config.rviz'],
            output='screen'
        )
    ])
