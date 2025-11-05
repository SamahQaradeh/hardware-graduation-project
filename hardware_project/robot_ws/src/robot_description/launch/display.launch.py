from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': open('/home/samah/robot_ws/src/robot_description/urdf/my_robot.urdf').read()
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/samah/robot_ws/src/map_config.rviz'],
            output='screen'
        )
        
        # ,
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_odom_to_base',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #     output='screen'
        # )

        
    ])
