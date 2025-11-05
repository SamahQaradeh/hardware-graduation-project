from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localization_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/localization_server']),
        ('share/localization_server', ['package.xml']),
        (os.path.join('share', 'localization_server', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'localization_server', 'config'), glob('config/*.yaml')),
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
             'initial_pose_pub = localization_server.initial_pose_pub:main'
        ],
    },
)
