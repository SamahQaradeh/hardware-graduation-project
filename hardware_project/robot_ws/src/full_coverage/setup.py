from setuptools import find_packages, setup

package_name = 'full_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav_client.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samah',
    maintainer_email='samah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner = full_coverage.coverage_planner:main',
            'visual = full_coverage.visualizer:main',
            
        ],
    },
)

