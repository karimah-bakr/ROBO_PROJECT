from setuptools import setup
from glob import glob
import os

package_name = 'turtlebot3_maze'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karem',
    maintainer_email='it.iu@outlook.com',
    description='Autonomous TurtleBot3 maze navigation + pick-and-place.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_navigator = turtlebot3_maze.maze_navigator:main',
            'lidar_processor = turtlebot3_maze.lidar_processor:main',
            'grid_mapper = turtlebot3_maze.grid_mapper:main',
            'path_planner = turtlebot3_maze.path_planner:main',
            'manipulator_controller = turtlebot3_maze.manipulator_controller:main',
            'standalone_navigator = turtlebot3_maze.standalone_navigator:main',
        ],
    },
)
