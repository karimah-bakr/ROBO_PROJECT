"""Mission FSM for the maze + pick-and-place demo.

States
------
    INIT            -- one-shot setup
    OBSERVE_CELL    -- wait for fresh /wall_status, push to /cell_observation
    EXPLORE         -- request BFS-to-nearest-unvisited from path_planner
    EXECUTE_MOVES   -- run the move queue (forward / turn) using odometry
    OBJECT_FOUND    -- object detected; remember the cell, plan pickup
    PICK_OBJECT     -- send /arm_command PICK, await /arm_status
    NAVIGATE_TARGET -- A* to target_cell, run moves
    PLACE_OBJECT    -- send /arm_command PLACE, await /arm_status
    RETURN_OBJECT2  -- A* back to the (now known) object cell
    RETURN_START    -- A* back to the spawn cell
    DONE            -- stop, log "Mission Complete"

Motion is dead-reckoned: the FSM keeps its own (row, col, heading) and
uses /odom only for incremental distance + yaw deltas between waypoints.
"""

from __future__ import annotations

import json
import math
import uuid
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


CELL_SIZE = 0.5

# heading: "N","E","S","W"
DR = {"N": 1, "S": -1, "E": 0, "W": 0}
DC = {"N": 0, "S": 0, "E": 1, "W": -1}
HEADINGS = ("N", "E", "S", "W")  # 90° CW step


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MazeNavigator(Node):
    def __init__(self) -> None:
        super().__init__("maze_navigator")

        self.declare_parameter("start_cell", [1, 3])
        self.declare_parameter("target_cell", [1, 7])
        self.declare_parameter("start_heading_rad", math.pi / 2)
        self.declare_parameter("cell_size_m", CELL_SIZE)
        self.declare_parameter("rows", 7)
        self.declare_parameter("cols", 7)
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("forward_tolerance_m", 0.01)
        self.declare_parameter("angular_tolerance_rad", 0.02)
        self.declare_parameter("max_explore_steps", 80)
        # LIDAR-based safe forward motion + corridor centering.
        self.declare_parameter("front_safety_dist_m", 0.20)
        self.declare_parameter("front_slow_dist_m", 0.30)
        self.declare_parameter("centering_gain", 1.2)
        self.declare_parameter("max_centering_ang", 0.30)
        self.declare_parameter("centering_max_side_dist_m", 0.32)

        sc = list(self.get_parameter("start_cell").value)
        tc = list(self.get_parameter("target_cell").value)
        self.start_cell: Tuple[int, int] = (int(sc[0]), int(sc[1]))
        self.target_cell: Tuple[int, int] = (int(tc[0]), int(tc[1]))
        # grid is 1..7 externally, but we use 0..6 internally
        self._start_internal = (self.start_cell[0] - 1, self.start_cell[1] - 1)
        self._target_internal = (self.target_cell[0] - 1, self.target_cell[1] - 1)

        # internal state
        self.row, self.col = self._start_internal
        self.heading = self._initial_heading_from_yaw(
            float(self.get_parameter("start_heading_rad").value)
        )
        self.cell_size = float(self.get_parameter("cell_size_m").value)
        self.lin_speed = float(self.get_parameter("linear_speed").value)
        self.ang_speed = float(self.get_parameter("angular_speed").value)
        self.fwd_tol = float(self.get_parameter("forward_tolerance_m").value)
        self.ang_tol = float(self.get_parameter("angular_tolerance_rad").value)
        self.max_explore = int(self.get_parameter("max_explore_steps").value)
        self.front_safety = float(self.get_parameter("front_safety_dist_m").value)
        self.front_slow = float(self.get_parameter("front_slow_dist_m").value)
        self.center_gain = float(self.get_parameter("centering_gain").value)
        self.center_max_ang = float(self.get_parameter("max_centering_ang").value)
        self.center_side_max = float(self.get_parameter("centering_max_side_dist_m").value)

        # FSM
        self.state = "INIT"
        self.move_queue: List[str] = []
        self.motion: Optional[dict] = None  # active forward/turn
        self.pending_request_id: Optional[str] = None
        self.latest_walls: Optional[dict] = None
        self.object_cell: Optional[Tuple[int, int]] = None
        self.objects_found: int = 0  # how many of the 2 we've handled
        self.first_object_cell: Optional[Tuple[int, int]] = None
        self.arm_in_flight: bool = False
        self.arm_last_ok: Optional[bool] = None
        self.explore_iters = 0
        self.last_obs_publish_cell: Optional[Tuple[int, int]] = None

        # odometry tracking
        self.have_odom = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.obs_pub = self.create_publisher(String, "/cell_observation", 10)
        self.req_pub = self.create_publisher(String, "/path_request", 10)
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self.alert_pub = self.create_publisher(String, "/maze_alert", 10)

        # subscribers
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor_qos)
        self.create_subscription(String, "/wall_status", self._on_wall_status, 10)
        self.create_subscription(String, "/object_detected", self._on_object, 10)
        self.create_subscription(String, "/path_result", self._on_path_result, 10)
        self.create_subscription(String, "/arm_status", self._on_arm_status, 10)

        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Robot initialized at cell ({self.row + 1},{self.col + 1})"
        )

    # ------------------------------------------------------------------
    # incoming

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.odom_x = p.position.x
        self.odom_y = p.position.y
        self.odom_yaw = _yaw_from_quat(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        )
        self.have_odom = True
        self._check_bounds()

    def _on_wall_status(self, msg: String) -> None:
        try:
            self.latest_walls = json.loads(msg.data)
        except Exception:
            self.latest_walls = None

    def _on_object(self, msg: String) -> None:
        if self.state in ("PICK_OBJECT", "NAVIGATE_TARGET", "PLACE_OBJECT",
                          "RETURN_OBJECT2", "RETURN_START", "DONE"):
            return
        if self.motion is not None:
            # ignore while moving; we re-check after each cell
            return
        try:
            data = json.loads(msg.data) if msg.data else {}
        except Exception:
            data = {}
        self.object_cell = (self.row, self.col)
        if self.first_object_cell is None:
            self.first_object_cell = self.object_cell
        self.get_logger().info(
            f"Found object at ({self.row + 1},{self.col + 1}) "
            f"(angle={data.get('angle_deg','?')}, dist={data.get('distance_m','?')})"
        )
        # cancel any pending exploration plan
        self.move_queue.clear()
        self.pending_request_id = None
        self._set_state("OBJECT_FOUND")

    def _on_path_result(self, msg: String) -> None:
        try:
            res = json.loads(msg.data)
        except Exception:
            return
        if self.pending_request_id is None or res.get("id") != self.pending_request_id:
            return
        self.pending_request_id = None
        if not res.get("ok"):
            self.get_logger().warn(f"planner: no path ({res.get('reason','')})")
            # In EXPLORE this means everything reachable is visited.
            if self.state == "EXPLORE":
                self.get_logger().warn("exploration exhausted without finding object")
                self._set_state("RETURN_START")
            return
        self.move_queue = list(res.get("moves", []))
        if not self.move_queue:
            # Already at goal — advance immediately.
            self._on_goal_reached_implicit()
            return
        self._set_state("EXECUTE_MOVES")

    def _on_arm_status(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
        except Exception:
            return
        if not self.arm_in_flight:
            return
        self.arm_in_flight = False
        self.arm_last_ok = bool(st.get("ok", False))
        self.get_logger().info(f"arm {st.get('cmd','?')} -> ok={self.arm_last_ok}")

    # ------------------------------------------------------------------
    # outgoing helpers

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _set_state(self, new_state: str) -> None:
        if new_state != self.state:
            self.get_logger().info(f"state: {self.state} -> {new_state}")
        self.state = new_state

    def _publish_observation(self) -> None:
        if self.latest_walls is None:
            return
        # Sectors → cardinal walls based on current heading.
        front = bool(self.latest_walls.get("F", False))
        right = bool(self.latest_walls.get("R", False))
        back = bool(self.latest_walls.get("B", False))
        left = bool(self.latest_walls.get("L", False))
        walls = self._sectors_to_cardinals(front, right, back, left)
        payload = {
            "row": self.row,
            "col": self.col,
            "heading": self.heading,
            "walls": walls,
        }
        self.obs_pub.publish(String(data=json.dumps(payload)))
        self.last_obs_publish_cell = (self.row, self.col)
        self.get_logger().info(f"Exploring cell ({self.row + 1},{self.col + 1})")

    def _sectors_to_cardinals(self, f: bool, r: bool, b: bool, l: bool) -> dict:
        order = {"N": 0, "E": 1, "S": 2, "W": 3}
        h_idx = order[self.heading]
        # mapping: sector at offset 0=F, 1=R(CW), 2=B, 3=L(CCW)
        # cardinal at h_idx = F, h_idx+1 (mod 4) = R, +2 = B, +3 = L
        cardinals = ["N", "E", "S", "W"]
        out = {}
        for offset, present in enumerate([f, r, b, l]):
            d = cardinals[(h_idx + offset) % 4]
            out[d] = bool(present)
        return out

    def _request_path(self, mode: str, goal: Optional[Tuple[int, int]] = None) -> None:
        self.pending_request_id = uuid.uuid4().hex[:8]
        req = {
            "id": self.pending_request_id,
            "mode": mode,
            "start": [self.row, self.col],
            "goal": list(goal) if goal else None,
            "heading": self.heading,
        }
        self.req_pub.publish(String(data=json.dumps(req)))

    def _check_bounds(self) -> None:
        # Bonus: out-of-maze alert
        if not self.have_odom:
            return
        x = self.odom_x
        y = self.odom_y
        if x < -0.05 or x > 3.55 or y < -0.05 or y > 3.55:
            self.alert_pub.publish(String(data="OUT_OF_MAZE"))

    # ------------------------------------------------------------------
    # main timer

    def _tick(self) -> None:
        try:
            self._tick_unsafe()
        except Exception as exc:
            self.get_logger().error(f"FSM tick crash: {exc}")
            self._stop()

    def _tick_unsafe(self) -> None:
        # If we have an active motion, advance it.
        if self.motion is not None:
            done = self._motion_step()
            if done:
                self._on_motion_done()
            return

        # No active motion -> next move or state action.
        if self.state == "INIT":
            self._set_state("OBSERVE_CELL")
            return

        if self.state == "OBSERVE_CELL":
            if self.latest_walls is None:
                return  # wait for first scan
            self._publish_observation()
            self._set_state("EXPLORE")
            return

        if self.state == "EXPLORE":
            self.explore_iters += 1
            if self.explore_iters > self.max_explore:
                self.get_logger().warn("explore step cap hit; giving up")
                self._set_state("RETURN_START")
                return
            if self.pending_request_id is None:
                self._request_path("explore")
            return

        if self.state == "EXECUTE_MOVES":
            if not self.move_queue:
                # Move list complete. Decide what comes after based on context.
                self._after_move_queue_done()
                return
            self._start_next_move()
            return

        if self.state == "OBJECT_FOUND":
            self.objects_found += 1
            label = f"{self.objects_found}/2"
            self.get_logger().info(f"Picking up object {label}")
            self.arm_pub.publish(String(data="PICK"))
            self.arm_in_flight = True
            self.arm_last_ok = None
            self._set_state("PICK_OBJECT")
            return

        if self.state == "PICK_OBJECT":
            if self.arm_in_flight:
                return
            self.get_logger().info(
                f"Navigating to target ({self.target_cell[0]},{self.target_cell[1]})"
            )
            self._set_state("NAVIGATE_TARGET")
            self._request_path("astar", self._target_internal)
            return

        if self.state == "NAVIGATE_TARGET":
            # Driven by /path_result -> EXECUTE_MOVES -> _after_move_queue_done()
            return

        if self.state == "PLACE_OBJECT":
            if self.arm_in_flight:
                return
            self.get_logger().info(
                f"Object {self.objects_found}/2 placed at "
                f"({self.target_cell[0]},{self.target_cell[1]})"
            )
            if self.objects_found < 2:
                self._set_state("RETURN_OBJECT2")
                self._request_path("astar", self.first_object_cell or (self.row, self.col))
            else:
                self._set_state("RETURN_START")
                self._request_path("astar", self._start_internal)
            return

        if self.state == "RETURN_OBJECT2":
            return  # driven by /path_result + EXECUTE_MOVES

        if self.state == "RETURN_START":
            return

        if self.state == "DONE":
            self._stop()
            return

    # ------------------------------------------------------------------
    # after the move queue empties

    def _after_move_queue_done(self) -> None:
        if self.state == "EXECUTE_MOVES":
            # Decide based on most recent high-level state.
            # The previous state is implicit; we tag it on entry into EXECUTE_MOVES.
            # For simplicity, look at what's still pending:
            if self.object_cell is not None and self.objects_found == 0:
                # We were exploring (and now happen to be elsewhere). Resume.
                self._set_state("EXPLORE")
                return
            # If we just navigated to target -> place
            if self.objects_found >= 1 and (self.row, self.col) == self._target_internal:
                # Tell the manipulator the exact world coords so it can drop the
                # carried object there (Gazebo set_entity_state inside the arm).
                tx = (self.target_cell[1] - 0.5) * self.cell_size
                ty = (self.target_cell[0] - 0.5) * self.cell_size
                self.arm_pub.publish(String(data=json.dumps(
                    {"cmd": "PLACE", "x": tx, "y": ty, "z": 0.1}
                )))
                self.arm_in_flight = True
                self.arm_last_ok = None
                self._set_state("PLACE_OBJECT")
                return
            # If we just returned to second object cell
            if self.objects_found == 1 and self.first_object_cell is not None \
                    and (self.row, self.col) == self.first_object_cell:
                self._set_state("OBJECT_FOUND")
                return
            # If we just returned home
            if (self.row, self.col) == self._start_internal and self.objects_found >= 2:
                self.get_logger().info(
                    f"Returning to start ({self.start_cell[0]},{self.start_cell[1]})"
                )
                self.get_logger().info("Mission Complete — all objects delivered")
                self._set_state("DONE")
                return
            # Fallback: re-observe + decide.
            self._set_state("OBSERVE_CELL")

    def _on_goal_reached_implicit(self) -> None:
        # Used when path planner returns an empty move list (already at goal).
        self._after_move_queue_done()

    # ------------------------------------------------------------------
    # motion primitives

    def _start_next_move(self) -> None:
        move = self.move_queue.pop(0)
        if move == "FORWARD":
            self.motion = {
                "type": "forward",
                "start_x": self.odom_x,
                "start_y": self.odom_y,
                "target_dist": self.cell_size,
            }
        elif move == "TURN_LEFT":
            self._begin_turn(+math.pi / 2.0)
        elif move == "TURN_RIGHT":
            self._begin_turn(-math.pi / 2.0)
        elif move == "TURN_180":
            self._begin_turn(math.pi)
        else:
            self.get_logger().warn(f"unknown move: {move}")

    def _begin_turn(self, signed_angle: float) -> None:
        self.motion = {
            "type": "turn",
            "start_yaw": self.odom_yaw,
            "target_delta": signed_angle,
            "accumulated": 0.0,
            "last_yaw": self.odom_yaw,
        }

    def _motion_step(self) -> bool:
        if not self.have_odom:
            return False
        m = self.motion
        assert m is not None
        twist = Twist()
        if m["type"] == "forward":
            dx = self.odom_x - m["start_x"]
            dy = self.odom_y - m["start_y"]
            dist = math.sqrt(dx * dx + dy * dy)
            remaining = m["target_dist"] - dist
            if remaining <= self.fwd_tol:
                self._stop()
                return True

            # LIDAR safety: if a wall is dangerously close ahead, bail out.
            fd = self._front_distance()
            if fd is not None and fd < self.front_safety:
                self.get_logger().warn(
                    f"front wall at {fd:.2f}m < safety {self.front_safety:.2f}m -- aborting forward"
                )
                m["aborted"] = True
                self._stop()
                return True

            # Speed: slow down when getting close to a wall ahead.
            speed = self.lin_speed
            if fd is not None and fd < self.front_slow:
                slow_ratio = max(
                    0.3,
                    (fd - self.front_safety) / max(self.front_slow - self.front_safety, 1e-3),
                )
                speed = self.lin_speed * slow_ratio

            twist.linear.x = speed
            twist.angular.z = self._centering_correction()
            self.cmd_pub.publish(twist)
            return False

        if m["type"] == "turn":
            d = _wrap(self.odom_yaw - m["last_yaw"])
            m["accumulated"] += d
            m["last_yaw"] = self.odom_yaw
            target = m["target_delta"]
            sign = 1.0 if target >= 0 else -1.0
            remaining = target - m["accumulated"]
            if abs(remaining) <= self.ang_tol:
                self._stop()
                return True
            twist.angular.z = sign * self.ang_speed
            self.cmd_pub.publish(twist)
            return False

        return True

    def _on_motion_done(self) -> None:
        m = self.motion
        self.motion = None
        if m is None:
            return
        if m["type"] == "forward":
            dx = self.odom_x - m["start_x"]
            dy = self.odom_y - m["start_y"]
            dist = math.sqrt(dx * dx + dy * dy)
            # If the forward step aborted before crossing the cell midline,
            # stay where we were and drop the rest of the queued moves --
            # the planner will replan from the actual position.
            if m.get("aborted") and dist < self.cell_size * 0.5:
                self.get_logger().info(
                    f"forward aborted at {dist:.2f}m (cell stays "
                    f"({self.row + 1},{self.col + 1})); requesting fresh plan"
                )
                self.move_queue.clear()
                self.pending_request_id = None
                self._publish_observation()
                # Force the FSM back into observation/plan rather than the
                # state we were running, so we don't dead-reckon into a wall.
                self._set_state("OBSERVE_CELL")
                return
            dr = DR[self.heading]
            dc = DC[self.heading]
            self.row += dr
            self.col += dc
            # observation refresh
            self._publish_observation()
        elif m["type"] == "turn":
            # update heading
            target = m["target_delta"]
            i = HEADINGS.index(self.heading)
            if abs(target - math.pi / 2.0) < 0.1:        # TURN_LEFT
                # CCW: N -> W -> S -> E -> N
                self.heading = HEADINGS[(i - 1) % 4]
            elif abs(target + math.pi / 2.0) < 0.1:      # TURN_RIGHT
                self.heading = HEADINGS[(i + 1) % 4]
            elif abs(target - math.pi) < 0.1:            # TURN_180
                self.heading = HEADINGS[(i + 2) % 4]

    # ------------------------------------------------------------------
    # LIDAR helpers used during forward motion

    def _front_distance(self) -> Optional[float]:
        if self.latest_walls is None:
            return None
        r = self.latest_walls.get("ranges", {}).get("F")
        if r is None:
            return None
        try:
            r = float(r)
        except (TypeError, ValueError):
            return None
        return None if not math.isfinite(r) else r

    def _centering_correction(self) -> float:
        """Small angular bias that keeps the robot midway between L/R walls."""
        if self.latest_walls is None:
            return 0.0
        ranges = self.latest_walls.get("ranges", {})
        l = ranges.get("L")
        r = ranges.get("R")
        try:
            l = float(l) if l is not None else None
            r = float(r) if r is not None else None
        except (TypeError, ValueError):
            return 0.0
        if l is None or r is None:
            return 0.0
        if not (math.isfinite(l) and math.isfinite(r)):
            return 0.0
        # Only correct when *both* side walls are present at a sensible range.
        if l > self.center_side_max or r > self.center_side_max:
            return 0.0
        # +err -> more clearance on left -> rotate left (+ang.z) to recenter.
        err = l - r
        ang = self.center_gain * err
        return max(-self.center_max_ang, min(self.center_max_ang, ang))

    # ------------------------------------------------------------------

    def _initial_heading_from_yaw(self, yaw: float) -> str:
        # snap to nearest cardinal
        yaw = _wrap(yaw)
        if -math.pi / 4 <= yaw < math.pi / 4:
            return "E"
        if math.pi / 4 <= yaw < 3 * math.pi / 4:
            return "N"
        if -3 * math.pi / 4 <= yaw < -math.pi / 4:
            return "S"
        return "W"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MazeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
