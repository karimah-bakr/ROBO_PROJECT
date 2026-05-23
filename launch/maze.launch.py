import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    pkg_maze = get_package_share_directory('turtlebot3_maze')
    world_path = os.path.join(pkg_maze, 'worlds', 'maze.world')

    manipulation_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_moveit_config'), 'launch')

    bringup_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_gazebo'), 'launch')

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [manipulation_launch_dir, '/move_group.launch.py']),
        launch_arguments={'use_sim': 'true'}.items(),
    )
    ld.add_action(move_group_launch)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [bringup_launch_dir, '/gazebo.launch.py']),
        launch_arguments={
            'world': world_path,
            'x_pose': '1.25',
            'y_pose': '0.75',
            'z_pose': '0.01',
            'roll': '0.00',
            'pitch': '0.00',
            'yaw': '0.00',
        }.items(),
    )
    ld.add_action(gazebo_launch)

    return ld
