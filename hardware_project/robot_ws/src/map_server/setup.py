from setuptools import find_packages, setup

package_name = 'map_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # remember to map the image and yaml file according to what I saved 
    data_files=[
        ('share/' + package_name + '/launch', ['launch/nav2_map_server.launch.py']),
        ('share/' + package_name + '/config', ['config/room_area.yaml', 'config/room_area.pgm']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
