from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare  # Correct import here
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Include the sllidar launch file in your robot_move launch
    sllidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('sllidar_ros2'), '/launch/view_sllidar_a1_launch.py']
        )
    )

    return LaunchDescription([
        sllidar_launch_file,  # Include sllidar launch
        Node(
            package='robot_move',
            executable='obstacle_detector',
            output='screen',
        )
        # ,
         
       
        # Node(
        #     package='robot_move',
        #     executable='read_vel_exe',
        #     output='screen',
        # )
        
        # ,
        # Node(
        #     package='robot_move',
        #     executable='send_vel_exe',
        #     output='screen',
        # )
        
        # ,
       
        # Node(
        #     package='robot_move',
        #     executable='read_imu_exe',
        #     output='screen',
        # ),
    ])


