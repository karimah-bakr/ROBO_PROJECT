#!/usr/bin/env python3
"""Gazebo bringup for TurtleBot3 manipulation in the maze world.

Same as upstream ``base.launch.py`` + ``gazebo.launch.py``, but:
  - uses ``turtlebot3_maze`` controller yaml (includes diff_drive_controller)
  - always spawns diff_drive_controller in simulation
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_maze = FindPackageShare("turtlebot3_maze")
    pkg_gazebo_ros = FindPackageShare("gazebo_ros")

    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_maze, "urdf", "turtlebot3_manipulation.urdf.xacro"]),
        " prefix:=", prefix,
        " use_sim:=", use_sim,
        " use_fake_hardware:=false",
        " fake_sensor_commands:=false",
    ])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_file, "use_sim_time": use_sim}],
        output="screen",
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzserver.launch.py"])
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzclient.launch.py"])
        ),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "turtlebot3_manipulation_system",
            "-x", x_pose, "-y", y_pose, "-z", z_pose,
            "-Y", yaw,
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
        output="screen",
    )
    imu_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[diff_drive_spawner, imu_spawner, arm_spawner, gripper_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("x_pose", default_value="1.25"),
        DeclareLaunchArgument("y_pose", default_value="0.25"),
        DeclareLaunchArgument("z_pose", default_value="0.01"),
        DeclareLaunchArgument("yaw", default_value="1.5708"),
        robot_state_pub,
        joint_state_spawner,
        after_joint_state,
        gzserver,
        gzclient,
        spawn_robot,
    ])
