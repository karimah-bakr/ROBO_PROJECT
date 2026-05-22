"""Launch maze world + TurtleBot3 with OpenManipulator-X (Gazebo).

Bringup loads diff_drive_controller in simulation (upstream yaml omits it).
Mission nodes start after the robot and controllers are ready.

Run:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch turtlebot3_maze maze_launch.py

Quick checks if the base does not move:
    ros2 topic echo /odom --once
    ros2 control list_controllers
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}" --once
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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
    tb3_manip_desc_share = get_package_share_directory(
        "turtlebot3_manipulation_description"
    )

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
        AppendEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            os.path.join(tb3_manip_desc_share, "urdf"),
            prepend=True,
        ),
    ]

    manip_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "manipulation_maze_bringup.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "use_sim": "true",
            "x_pose": spawn_x,
            "y_pose": spawn_y,
            "z_pose": "0.01",
            "yaw": spawn_yaw,
        }.items(),
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
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="1.25"),
        DeclareLaunchArgument("spawn_y", default_value="0.25"),
        DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
        DeclareLaunchArgument("rviz", default_value="false"),
        *set_model_path,
        manip_bringup,
        TimerAction(period=20.0, actions=[lidar, mapper, planner, arm, navigator]),
        rviz,
    ])
