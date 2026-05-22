"""Launch maze world + TurtleBot3 with OpenManipulator-X (Gazebo).

Required:
    export TURTLEBOT3_MODEL=waffle_pi
    sudo apt install ros-humble-turtlebot3-gazebo \\
                     ros-humble-turtlebot3-manipulation \\
                     ros-humble-turtlebot3-manipulation-gazebo

Run:
    ros2 launch turtlebot3_maze maze_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Manipulation is designed for Waffle Pi; set default if user forgot.
    os.environ.setdefault("TURTLEBOT3_MODEL", "waffle_pi")

    pkg_share = get_package_share_directory("turtlebot3_maze")
    world_path = os.path.join(pkg_share, "worlds", "maze_world.world")
    params_path = os.path.join(pkg_share, "config", "maze_params.yaml")
    rviz_path = os.path.join(pkg_share, "rviz", "maze_view.rviz")

    use_sim_time = {"use_sim_time": True}

    spawn_x = LaunchConfiguration("spawn_x", default="1.25")
    spawn_y = LaunchConfiguration("spawn_y", default="0.25")
    spawn_yaw = LaunchConfiguration("spawn_yaw", default="1.5708")
    use_rviz = LaunchConfiguration("rviz", default="false")

    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_manip_share = get_package_share_directory("turtlebot3_manipulation_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # So Gazebo finds TB3 + manipulator meshes.
    set_model_path = [
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(tb3_gazebo_share, "models"),
            prepend=True,
        ),
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(tb3_manip_share, "models"),
            prepend=True,
        ),
    ]

    # ros2_control + robot_state_publisher (use_sim:=true → Gazebo hardware).
    manip_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_manip_share, "launch", "base.launch.py")
        ),
        launch_arguments={
            "use_sim": "true",
            "start_rviz": "false",
        }.items(),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
    )

    # Spawn full manipulation URDF (base + arm + gripper + lidar plugins).
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "turtlebot3_manipulation_system",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.01",
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    lidar = Node(
        package="turtlebot3_maze",
        executable="lidar_processor",
        name="lidar_processor",
        output="screen",
        parameters=[params_path, use_sim_time],
    )
    mapper = Node(
        package="turtlebot3_maze",
        executable="grid_mapper",
        name="grid_mapper",
        output="screen",
        parameters=[params_path, use_sim_time],
    )
    planner = Node(
        package="turtlebot3_maze",
        executable="path_planner",
        name="path_planner",
        output="screen",
        parameters=[params_path, use_sim_time],
    )
    arm = Node(
        package="turtlebot3_maze",
        executable="manipulator_controller",
        name="manipulator_controller",
        output="screen",
        parameters=[params_path, use_sim_time],
    )
    navigator = Node(
        package="turtlebot3_maze",
        executable="standalone_navigator",
        name="standalone_navigator",
        output="screen",
        parameters=[params_path, use_sim_time],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[use_sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="1.25"),
        DeclareLaunchArgument("spawn_y", default_value="0.25"),
        DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
        DeclareLaunchArgument("rviz", default_value="false"),
        *set_model_path,
        gzserver,
        gzclient,
        manip_base,
        TimerAction(period=4.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[lidar, mapper, planner, arm]),
        TimerAction(period=8.0, actions=[navigator]),
        TimerAction(period=2.0, actions=[rviz]),
    ])
