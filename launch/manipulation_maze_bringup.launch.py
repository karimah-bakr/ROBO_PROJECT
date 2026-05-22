#!/usr/bin/env python3
"""Gazebo bringup for TurtleBot3 manipulation in the maze world.

Controller spawners start only AFTER the robot is spawned in Gazebo (so
gazebo_ros2_control creates /controller_manager first).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        parameters=[{
            "robot_description": ParameterValue(urdf_file, value_type=str),
            "use_sim_time": use_sim,
        }],
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

    # Spawn robot after Gazebo is up, then load all controllers in parallel.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_robot])
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_spawner,
                diff_drive_spawner,
                imu_spawner,
                arm_spawner,
                gripper_spawner,
            ],
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
        gzserver,
        gzclient,
        robot_state_pub,
        delayed_spawn,
        load_controllers,
    ])
