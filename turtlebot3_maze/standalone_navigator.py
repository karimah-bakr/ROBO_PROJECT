#!/usr/bin/env python3
"""Standalone maze mission navigator (TurtleBot3 + OpenManipulator-X).

Mission
-------
    (1,3) start -> (5,1) pick object_1 -> (1,7) place
                -> (5,1) pick object_2 -> (1,7) place
                -> (1,3) return -> done

The script does everything itself:
    - BFS on the embedded maze layout (must match worlds/maze_world.world)
    - cell-by-cell motion: turn in place, then drive one cell forward
    - /odom for distance + yaw tracking, /scan for front-safety abort
    - /arm_command (JSON) for pick/place, /arm_status to wait for completion

How to run
----------
Already wired into `launch/maze_launch.py` as the `standalone_navigator`
node (replaces the old FSM `maze_navigator`). After rebuilding:

    colcon build --packages-select turtlebot3_maze
    source install/setup.bash
    ros2 launch turtlebot3_maze maze_launch.py
"""

from __future__ import annotations

import json
import math
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# =============================================================================
# Maze layout — MUST match worlds/maze_world.world (regenerate together).
# H_WALLS[y][x] = wall on south edge of cell (y+1, x+1)
# V_WALLS[x][y] = wall on west  edge of cell (y+1, x+1)
# Row 1 = south, col 1 = west, both 1..7.
# =============================================================================
H_WALLS = [
    [1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 1, 0, 0],   # H[1][2]=0 -> opens north exit of start cell (1,3)
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1],
    [0, 0, 1, 1, 1, 1, 1],
]
V_WALLS = [
    [1, 1, 1, 1, 1, 1, 1],
    [0, 1, 0, 1, 0, 1, 0],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1],
]

CELL = 0.5
ROWS = 7
COLS = 7

START_CELL  = (1, 3)
OBJECT_CELL = (5, 1)
TARGET_CELL = (1, 7)

# Initial heading from launch: spawn_yaw = pi/2 -> facing North (+Y).
INITIAL_HEADING = "N"

# heading deltas
DR = {"N":  1, "S": -1, "E": 0, "W":  0}
DC = {"N":  0, "S":  0, "E": 1, "W": -1}
YAW = {"N": math.pi / 2, "S": -math.pi / 2, "E": 0.0, "W": math.pi}
HEADINGS = ("N", "E", "S", "W")  # 90° clockwise


# =============================================================================
# Maze helpers
# =============================================================================

def has_wall(r: int, c: int, d: str) -> bool:
    if d == "N": return bool(H_WALLS[r][c - 1])
    if d == "S": return bool(H_WALLS[r - 1][c - 1])
    if d == "E": return bool(V_WALLS[c][r - 1])
    if d == "W": return bool(V_WALLS[c - 1][r - 1])
    return True


def neighbours(r: int, c: int):
    for d in "NESW":
        if has_wall(r, c, d):
            continue
        nr, nc = r + DR[d], c + DC[d]
        if 1 <= nr <= ROWS and 1 <= nc <= COLS:
            yield (nr, nc), d


def bfs(start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    if start == goal:
        return [start]
    came: Dict[Tuple[int, int], Tuple[int, int]] = {}
    seen = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            path = [cur]
            while path[-1] != start:
                path.append(came[path[-1]])
            return list(reversed(path))
        for nb, _ in neighbours(*cur):
            if nb not in seen:
                seen.add(nb)
                came[nb] = cur
                q.append(nb)
    return None


def cells_to_steps(cells: List[Tuple[int, int]]) -> List[str]:
    """Return the cardinal-heading list, one entry per FORWARD step."""
    steps: List[str] = []
    for (r0, c0), (r1, c1) in zip(cells[:-1], cells[1:]):
        for d in "NESW":
            if r1 - r0 == DR[d] and c1 - c0 == DC[d]:
                steps.append(d)
                break
    return steps


def cell_center(rc: Tuple[int, int]) -> Tuple[float, float]:
    r, c = rc
    return (c - 0.5) * CELL, (r - 0.5) * CELL


def wrap(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# =============================================================================
# Navigator node
# =============================================================================

class MazeMissionNavigator(Node):
    # locomotion
    LIN_SPEED      = 0.12   # m/s during straight drive
    ANG_SPEED      = 0.7    # rad/s during in-place rotation
    FWD_TOL        = 0.02   # m -- stop band on the 0.5 m forward leg
    ANG_TOL        = 0.03   # rad -- stop band on a turn
    FRONT_SAFETY   = 0.12   # m -- abort forward if front sector < this
    FRONT_SLOW     = 0.22   # m -- start slowing down below this
    SCAN_FRONT_DEG = 15.0   # ± window around 0° for the safety check
    MIN_FWD_BEFORE_LIDAR_ABORT = 0.08

    def __init__(self) -> None:
        super().__init__("maze_mission_navigator")

        # Gazebo publishes /odom as reliable; /scan as sensor (best effort).
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)

        # subscribers
        self.create_subscription(Odometry,  "/odom", self._on_odom, odom_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String,    "/arm_status", self._on_arm,    10)

        # odom state
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # lidar state
        self.front_dist: float = float("inf")

        # arm state
        self.arm_busy = False
        self.arm_last_ok: Optional[bool] = None

        # robot pose in the grid
        self.cell = START_CELL
        self.heading = INITIAL_HEADING

        # mission script: ordered list of (kind, payload)
        # kinds: "goto" -> goal_cell ; "pick" -> None ; "place" -> (x,y,z)
        target_xy = cell_center(TARGET_CELL)
        self.script: List[Tuple[str, object]] = [
            ("goto",  OBJECT_CELL),
            ("pick",  None),
            ("goto",  TARGET_CELL),
            ("place", (target_xy[0], target_xy[1], 0.10)),
            ("goto",  OBJECT_CELL),
            ("pick",  None),
            ("goto",  TARGET_CELL),
            ("place", (target_xy[0], target_xy[1], 0.10)),
            ("goto",  START_CELL),
            ("done",  None),
        ]
        self.script_i = 0

        # active sub-action state
        # for "goto": queued cardinal-heading steps to consume
        self.step_queue: List[str] = []
        # current low-level motion: None | {"type":"turn"|"forward", ...}
        self.motion: Optional[dict] = None

        # logging guards
        self._waiting_logged = False
        self._scan_count = 0
        self._odom_count = 0

        self.create_timer(0.05, self._tick)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(
            f"standalone_navigator ready: "
            f"start={START_CELL} object={OBJECT_CELL} target={TARGET_CELL}"
        )

    def _heartbeat(self) -> None:
        kind = self.script[self.script_i][0] if self.script_i < len(self.script) else "idle"
        self.get_logger().info(
            f"hb: cell={self.cell} heading={self.heading} step={kind} "
            f"odom={self._odom_count} scan={self._scan_count} "
            f"front={self.front_dist:.2f}m"
        )

    # -------------------------------------------------------------------------
    # subscribers
    # -------------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.have_odom = True
        self._odom_count += 1

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan_count += 1
        # Take the minimum range in ± SCAN_FRONT_DEG around 0 rad.
        if not msg.ranges:
            return
        n = len(msg.ranges)
        half = math.radians(self.SCAN_FRONT_DEG)
        rmin = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < 0.06:
                continue
            ang = wrap(msg.angle_min + i * msg.angle_increment)
            if -half <= ang <= half and r < rmin:
                rmin = r
        self.front_dist = rmin

    def _on_arm(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not self.arm_busy:
            return
        self.arm_busy = False
        self.arm_last_ok = bool(data.get("ok", False))
        self.get_logger().info(
            f"arm {data.get('cmd','?')} -> ok={self.arm_last_ok}"
        )

    # -------------------------------------------------------------------------
    # main loop
    # -------------------------------------------------------------------------

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _tick(self) -> None:
        if not self.have_odom:
            if not self._waiting_logged:
                self.get_logger().info("waiting for /odom...")
                self._waiting_logged = True
            return

        # 1) drive any active low-level motion
        if self.motion is not None:
            if self._motion_step():
                self._motion_done()
            return

        # 2) drain any queued cell-step
        if self.step_queue:
            self._start_next_step()
            return

        # 3) advance the high-level script
        if self.script_i >= len(self.script):
            self._stop()
            return

        kind, payload = self.script[self.script_i]

        if kind == "done":
            self._stop()
            self.get_logger().info("MISSION COMPLETE")
            self.script_i += 1
            return

        if kind == "goto":
            goal = payload  # type: ignore[assignment]
            assert isinstance(goal, tuple)
            if self.cell == goal:
                self.script_i += 1
                return
            cells = bfs(self.cell, goal)
            if not cells:
                self.get_logger().error(f"no path {self.cell} -> {goal}; aborting")
                self.script_i = len(self.script)
                return
            steps = cells_to_steps(cells)
            self.get_logger().info(
                f"plan {self.cell}->{goal}: {len(steps)} steps via {cells}"
            )
            self.step_queue = steps
            return

        if kind == "pick":
            ox, oy = cell_center(OBJECT_CELL)
            self._arm_step(
                "pick",
                must_be_at=OBJECT_CELL,
                payload={"cmd": "PICK", "x": ox, "y": oy, "z": 0.10},
            )
            return

        if kind == "place":
            x, y, z = payload  # type: ignore[misc]
            self._arm_step(
                "place",
                must_be_at=TARGET_CELL,
                payload={"cmd": "PLACE", "x": x, "y": y, "z": z},
            )
            return

    # -------------------------------------------------------------------------
    # low-level motion primitives
    # -------------------------------------------------------------------------

    def _start_next_step(self) -> None:
        d = self.step_queue[0]
        # 1) align heading if needed
        if d != self.heading:
            self.motion = {
                "type":       "turn",
                "target_yaw": YAW[d],
                "next_heading": d,
            }
            return
        # 2) forward by one cell
        self.motion = {
            "type":     "forward",
            "start_x":  self.x,
            "start_y":  self.y,
            "target":   CELL,
            "next_cell": (self.cell[0] + DR[d], self.cell[1] + DC[d]),
        }

    def _motion_step(self) -> bool:
        m = self.motion
        assert m is not None
        twist = Twist()

        if m["type"] == "turn":
            err = wrap(m["target_yaw"] - self.yaw)
            if abs(err) <= self.ANG_TOL:
                self._stop()
                return True
            twist.angular.z = self.ANG_SPEED if err > 0 else -self.ANG_SPEED
            self.cmd_pub.publish(twist)
            return False

        # forward
        dx = self.x - m["start_x"]
        dy = self.y - m["start_y"]
        dist = math.hypot(dx, dy)
        remaining = m["target"] - dist

        if remaining <= self.FWD_TOL:
            self._stop()
            return True

        if dist >= self.MIN_FWD_BEFORE_LIDAR_ABORT and self.front_dist < self.FRONT_SAFETY:
            self.get_logger().warn(
                f"front={self.front_dist:.2f}m < safety {self.FRONT_SAFETY:.2f}m; "
                f"aborting forward at dist={dist:.2f}m"
            )
            self._stop()
            m["aborted"] = True
            return True

        # Slow down when approaching a wall (or end of cell).
        speed = self.LIN_SPEED
        if self.front_dist < self.FRONT_SLOW:
            ratio = max(0.3, (self.front_dist - self.FRONT_SAFETY)
                              / max(self.FRONT_SLOW - self.FRONT_SAFETY, 1e-3))
            speed *= ratio

        # tiny yaw correction so we stay on the intended cardinal heading
        yaw_err = wrap(YAW[self.heading] - self.yaw)
        twist.linear.x = speed
        twist.angular.z = max(-0.4, min(0.4, 1.5 * yaw_err))
        self.cmd_pub.publish(twist)
        return False

    def _motion_done(self) -> None:
        m = self.motion
        self.motion = None
        if m is None:
            return

        if m["type"] == "turn":
            self.heading = m["next_heading"]
            return

        # forward
        if m.get("aborted"):
            # Drop the rest of the queued plan and let the next tick re-plan
            # from the (unchanged) current cell.
            self.get_logger().warn("forward aborted; re-planning from current cell")
            self.step_queue.clear()
            return

        # Consumed one step successfully.
        self.cell = m["next_cell"]
        self.step_queue.pop(0)
        self.get_logger().info(f"reached cell {self.cell}, heading {self.heading}")

    def _arm_step(
        self,
        label: str,
        must_be_at: Tuple[int, int],
        payload: dict,
    ) -> None:
        if self.cell != must_be_at:
            cells = bfs(self.cell, must_be_at)
            if not cells:
                self.get_logger().error(f"no path to {must_be_at} for {label}")
                self.script_i = len(self.script)
                return
            self.step_queue = cells_to_steps(cells)
            return
        if self.arm_busy:
            return
        if self.arm_last_ok is None:
            self.get_logger().info(f"{label.upper()} at {self.cell}: {payload}")
            self.arm_busy = True
            self.arm_pub.publish(String(data=json.dumps(payload)))
            return
        if self.arm_last_ok:
            self.arm_last_ok = None
            self.script_i += 1
        else:
            self.get_logger().error(f"{label} failed — retry")
            self.arm_last_ok = None
            self.arm_busy = True
            self.arm_pub.publish(String(data=json.dumps(payload)))


# =============================================================================
# entry point
# =============================================================================

def main() -> None:
    rclpy.init()
    node = MazeMissionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
