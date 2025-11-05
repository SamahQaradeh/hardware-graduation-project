from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='full_coverage',
            executable='coverage_planner',
            name='coverage_planner',
            output='screen'
        ),
         Node(
            package='full_coverage',
            executable='visual',
            name='visual_node',
            output='screen'
        ),





    ])