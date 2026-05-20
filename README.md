# turtlebot3_maze

Autonomous TurtleBot3 Burger + OpenManipulator-X solving a 7×7 maze:
explore → find 2 objects → pick → deliver to target → return.

## Layout

```
turtlebot3_maze/
├── launch/maze_launch.py
├── worlds/maze_world.world
├── config/maze_params.yaml
├── rviz/maze_view.rviz
├── turtlebot3_maze/
│   ├── maze_navigator.py        # FSM mission controller
│   ├── lidar_processor.py       # /scan -> /wall_status, /object_detected
│   ├── grid_mapper.py           # 7x7 occupancy + /maze_map publisher
│   ├── path_planner.py          # BFS exploration + A* navigation
│   └── manipulator_controller.py# OpenManipulator-X pick/place
├── package.xml
├── setup.py
└── setup.cfg
```

## Coordinate frame

- World origin = bottom-left corner of cell (1, 1).
- +X = east (column increases), +Y = north (row increases).
- Cell (r, c) centre = `((c-0.5) * 0.5, (r-0.5) * 0.5)` metres.
- Robot spawns at cell (1, 3) → (1.25, 0.25) m, yaw = π/2 (faces +Y).

## Maze features (designed into `maze_world.world`)

- **Forced exploration:** east of the start cell is walled, so the robot
  must drive north first.
- **Dead-end alley:** row 1 cols 4–6 are reachable only by descending
  from row 2 col 7.
- **Long corridor:** row 4 / row 5 separated by three wall segments with
  two gaps; the resulting corridor along row 5 is 7 cells wide.
- **T-junction:** an east-west wall north of (3, 3)/(3, 4) with a stub
  jutting north creates a textbook T at (3, 4).
- **Loop:** the 2×2 cells (5, 5) – (5, 6) – (6, 5) – (6, 6) have no
  internal walls, so the planner can route around them either way.
- **Objects:** two cubes (red + blue, 0.02 × 0.03 × 0.20 m) at the
  centre of cell (5, 3).

## Build & run (on The Construct or any ROS 2 Humble workstation)

```bash
# 1. drop the package into your workspace's src/
cp -r turtlebot3_maze ~/ros2_ws/src/

# 2. build
cd ~/ros2_ws
colcon build --packages-select turtlebot3_maze
source install/setup.bash

# 3. environment (Burger model)
export TURTLEBOT3_MODEL=burger

# 4. launch (defaults: start_cell=[1,3], target_cell=[1,7])
ros2 launch turtlebot3_maze maze_launch.py

# 5. override start/target at runtime
ros2 launch turtlebot3_maze maze_launch.py start_cell:="[1,3]" target_cell:="[1,7]"
```

The expected console trace looks like:

```
Robot initialized at cell (1,3)
Exploring cell (1,3)
Wall detected: EAST at (1,3)
Exploring cell (2,3)
...
Found object at (5,3)
Picking up object 1/2
Navigating to target (1,7)
Object 1/2 placed at (1,7)
Picking up object 2/2
Navigating to target (1,7)
Object 2/2 placed at (1,7)
Returning to start (1,3)
Mission Complete — all objects delivered
```

## Topic graph

| Publisher                | Topic                | Type        | Purpose                              |
| ------------------------ | -------------------- | ----------- | ------------------------------------ |
| `lidar_processor`        | `/wall_status`       | String JSON | per-sector wall flags + ranges       |
| `lidar_processor`        | `/object_detected`   | String JSON | front-sector cluster match           |
| `maze_navigator`         | `/cell_observation`  | String JSON | cell + cardinal walls (heading-aware)|
| `grid_mapper`            | `/maze_map`          | String JSON | full discovered map snapshot         |
| `maze_navigator`         | `/path_request`      | String JSON | BFS-explore or A* goal               |
| `path_planner`           | `/path_result`       | String JSON | move list + cell list                |
| `maze_navigator`         | `/arm_command`       | String      | "PICK" / "PLACE" / "HOME"            |
| `manipulator_controller` | `/arm_status`        | String JSON | completion + ok flag                 |
| `maze_navigator`         | `/cmd_vel`           | Twist       | low-level velocity                   |
| `maze_navigator`         | `/maze_alert`        | String      | "OUT_OF_MAZE" if odom escapes box    |

## Tuning knobs

Everything lives in `config/maze_params.yaml`:

- `linear_speed`, `angular_speed` — cap is 0.10 m/s and 0.5 rad/s per the spec.
- `wall_threshold_m` — distance below which a sector counts as walled (0.25 m default).
- `object_*` — width/length window for the cluster detector. Loosen if
  Gazebo LIDAR misses the thin cubes.
- `home_joints` / `reach_joints` / `lower_joints` / `carry_joints` — adjust
  if the OpenManipulator URDF in your bringup uses different limits.

## Bonus features included

- **RViz config** (`rviz/maze_view.rviz`): top-down view, robot model,
  laser, odometry trail, 7×7 grid centred at (1.75, 1.75).
- **Boundary watchdog:** `maze_navigator` publishes `"OUT_OF_MAZE"` on
  `/maze_alert` if odometry drifts outside `[-0.05, 3.55] m` on X or Y.
  Hook this up to a recovery behaviour if needed.

## Recording a demo video

Two options.

### A. Gazebo built-in recorder (no extra packages)

```bash
ros2 launch turtlebot3_maze maze_launch.py &
# inside Gazebo: View -> Recording -> Start
# the .mp4 lands under ~/.gazebo/log/
```

### B. ffmpeg screen capture (recommended on The Construct)

```bash
# in one terminal:
ros2 launch turtlebot3_maze maze_launch.py
# in another, capture the whole screen for 90 s at 30 fps:
ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
       -t 90 -c:v libx264 -preset veryfast maze_demo.mp4
```

### C. RViz only (cleanest for grading)

If you only need to show the discovered map + the path trail:

```bash
ros2 launch turtlebot3_maze maze_launch.py rviz:=true
# then "Window -> Recording" inside RViz (Tools menu varies by version)
```

## Notes / limitations

- Movement is dead-reckoned from odometry deltas; cumulative drift over
  ~25 cells is small but visible. If you tighten the tolerances, also
  lower `linear_speed` to give the controller time to settle.
- `manipulator_controller` falls back to a stub if `open_manipulator_msgs`
  isn't installed in the workspace — it will log the would-be commands
  and the FSM still progresses end-to-end. With the real package
  installed, the joint service calls are issued normally.
- Nav2 is intentionally *not* used; navigation, mapping and planning are
  all in-package as the project brief required.
