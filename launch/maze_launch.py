"""Launch the full maze + pick-and-place demo.

Order:
  1. Gazebo with worlds/maze_world.world
  2. robot_state_publisher (TurtleBot3 + OpenManipulator URDF)
  3. spawn the robot at start_cell
  4. lidar_processor
  5. grid_mapper
  6. path_planner
  7. manipulator_controller
  8. maze_navigator  (the FSM)

Required environment:
    export TURTLEBOT3_MODEL=burger        # or waffle / waffle_pi

Install on the workstation:
    sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description

Override params at launch time:
    ros2 launch turtlebot3_maze maze_launch.py start_cell:=[1,3] target_cell:=[1,7]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("turtlebot3_maze")
    world_path = os.path.join(pkg_share, "worlds", "maze_world.world")
    params_path = os.path.join(pkg_share, "config", "maze_params.yaml")
    rviz_path = os.path.join(pkg_share, "rviz", "maze_view.rviz")

    use_sim_time = {"use_sim_time": True}

    # cell (1,3) center = ((3-0.5)*0.5, (1-0.5)*0.5) = (1.25, 0.25)
    spawn_x = LaunchConfiguration("spawn_x", default="1.25")
    spawn_y = LaunchConfiguration("spawn_y", default="0.25")
    spawn_yaw = LaunchConfiguration("spawn_yaw", default="1.5708")  # pi/2, +Y
    start_cell = LaunchConfiguration("start_cell", default="[1, 3]")
    target_cell = LaunchConfiguration("target_cell", default="[1, 7]")
    use_rviz = LaunchConfiguration("rviz", default="false")

    # 1) Gazebo with the maze world.
    gazebo_ros = get_package_share_directory("gazebo_ros")
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py")),
    )

    # 2) TurtleBot3 model — MUST use turtlebot3_gazebo SDF (has /odom + /scan plugins).
    # Spawning plain URDF from turtlebot3_description gives a static shell with no sensors.
    model = os.environ.get("TURTLEBOT3_MODEL", "burger")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    model_path = os.path.join(tb3_gazebo, "models", f"turtlebot3_{model}", "model.sdf")
    if not os.path.isfile(model_path):
        raise FileNotFoundError(
            f"TurtleBot3 Gazebo model not found: {model_path}\n"
            "Install: sudo apt install ros-humble-turtlebot3-gazebo\n"
            f"And set: export TURTLEBOT3_MODEL=burger  (you have: {model})"
        )

    gazebo_models_path = os.path.join(tb3_gazebo, "models")
    set_gazebo_model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        gazebo_models_path,
        prepend=True,
    )

    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        f"turtlebot3_{model}.urdf",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Command(["xacro ", urdf_path])},
            use_sim_time,
        ],
    )

    # 3) Spawn TurtleBot3 with Gazebo plugins (diff drive + LIDAR).
    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", f"turtlebot3_{model}",
            "-file", model_path,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", "0.01",
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    # 4-7) custom nodes.
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
        parameters=[use_sim_time],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        condition=None,  # always include; user can disable by closing the window
        parameters=[use_sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument("spawn_x", default_value="1.25"),
        DeclareLaunchArgument("spawn_y", default_value="0.25"),
        DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
        DeclareLaunchArgument("start_cell", default_value="[1, 3]"),
        DeclareLaunchArgument("target_cell", default_value="[1, 7]"),
        DeclareLaunchArgument("rviz", default_value="false"),

        set_gazebo_model_path,
        gzserver,
        gzclient,
        rsp,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=5.0, actions=[lidar, mapper, planner, arm]),
        TimerAction(period=7.0, actions=[navigator]),
        TimerAction(period=2.0, actions=[rviz]),
    ])
