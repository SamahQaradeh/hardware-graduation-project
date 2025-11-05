from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'robot_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = robot_move.robot_move:main',
            'read_vel_exe= robot_move.read_vel_from_ard:main',
            'send_vel_exe = robot_move.send_vel_to_ard:main',
            'read_imu_exe=robot_move.read_imu_from_ard:main',
            'send_point=robot_move.send_point_to_point:main'
        ],
    },
)
