"""OpenManipulator-X controller for TurtleBot3 manipulation (Gazebo / hardware).

/arm_command  JSON or plain text:
    {"cmd": "PICK",  "x": 0.25, "y": 2.25, "z": 0.1, "joint1": 3.14}
    {"cmd": "PLACE", "x": 3.25, "y": 0.25, "z": 0.1, "joint1": 0.0}
    "HOME"

/arm_status   {"cmd": "...", "ok": true|false, "reason": "..."}

Backends (first match wins):
  1. open_manipulator_msgs services (legacy bringup)
  2. ros2_control actions (turtlebot3_manipulation_gazebo)
  3. timed stub + optional Gazebo object teleport
"""

from __future__ import annotations

import json
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

try:
    from open_manipulator_msgs.srv import SetJointPosition  # type: ignore
    HAVE_OM = True
except Exception:
    SetJointPosition = None  # type: ignore
    HAVE_OM = False

try:
    from gazebo_msgs.srv import SetEntityState  # type: ignore
    HAVE_GZ = True
except Exception:
    SetEntityState = None  # type: ignore
    HAVE_GZ = False

try:
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    HAVE_RC = True
except Exception:
    FollowJointTrajectory = None  # type: ignore
    GripperCommand = None  # type: ignore
    HAVE_RC = False

from builtin_interfaces.msg import Duration


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
OBJECT_NAMES = ("object_1", "object_2")
STOW_Z = -2.0
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
GZ_SERVICE_NAMES = (
    "/gazebo/set_entity_state",
    "/set_entity_state",
)


class ManipulatorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_controller")

        # Joint angles from the working course reference model. joint1 stays
        # at 0 — chassis is already turned to face the cube by the navigator,
        # so the arm reaches forward in the robot's local frame.
        self.declare_parameter("home_joints",  [0.0,  0.0,  0.0,  0.0])
        self.declare_parameter("reach_joints", [0.0,  0.6, -0.2, -0.4])
        self.declare_parameter("lower_joints", [0.0,  0.9, -0.3, -0.5])
        self.declare_parameter("carry_joints", [0.0,  0.0,  0.0,  0.0])
        self.declare_parameter("gripper_open",  0.019)
        self.declare_parameter("gripper_close", -0.019)
        self.declare_parameter("step_wait_s",   4.0)
        # Chassis micro-moves during pick (taken from the working course
        # reference). Back off briefly so the wrist has room to swing down,
        # then nudge forward to wedge the cube between the closing fingers.
        self.declare_parameter("pick_back_speed_mps", 0.02)
        self.declare_parameter("pick_back_seconds",   1.5)
        self.declare_parameter("pick_fwd_speed_mps",  0.02)
        self.declare_parameter("pick_fwd_seconds",    1.0)
        self.declare_parameter("require_gazebo_teleport", True)
        self.declare_parameter("gazebo_service_wait_s", 15.0)

        self.home = list(self.get_parameter("home_joints").value)
        self.reach = list(self.get_parameter("reach_joints").value)
        self.lower = list(self.get_parameter("lower_joints").value)
        self.carry = list(self.get_parameter("carry_joints").value)
        self.g_open = float(self.get_parameter("gripper_open").value)
        self.g_close = float(self.get_parameter("gripper_close").value)
        self.step_wait = float(self.get_parameter("step_wait_s").value)
        self.pick_back_v = float(self.get_parameter("pick_back_speed_mps").value)
        self.pick_back_t = float(self.get_parameter("pick_back_seconds").value)
        self.pick_fwd_v  = float(self.get_parameter("pick_fwd_speed_mps").value)
        self.pick_fwd_t  = float(self.get_parameter("pick_fwd_seconds").value)
        self.require_tp = bool(self.get_parameter("require_gazebo_teleport").value)
        self._gz_wait = float(self.get_parameter("gazebo_service_wait_s").value)

        self.cmd_sub = self.create_subscription(String, "/arm_command", self._on_cmd, 5)
        self.status_pub = self.create_publisher(String, "/arm_status", 10)
        # /cmd_vel for the back/forward chassis nudge during pick. Navigator
        # holds station (arm_busy guard) while the arm is running so this
        # doesn't fight a concurrent driving command.
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._joint_cli = None
        self._tool_cli = None
        if HAVE_OM:
            self._joint_cli = self.create_client(
                SetJointPosition, "/open_manipulator/goal_joint_space_path"
            )
            self._tool_cli = self.create_client(
                SetJointPosition, "/open_manipulator/goal_tool_control"
            )

        self._arm_action = None
        self._gripper_action = None
        if HAVE_RC:
            self._arm_action = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
            self._gripper_action = ActionClient(self, GripperCommand, GRIPPER_ACTION)

        self._gz_cli = None
        self._gz_service_name = ""
        self._init_gazebo_client()

        self._next_object_idx = 0
        self._held_object: Optional[str] = None
        self._busy = threading.Lock()
        self._pending_joint1: Optional[float] = None

        self.get_logger().info(
            "manipulator_controller ready "
            f"(open_manipulator={HAVE_OM}, ros2_control={HAVE_RC}, "
            f"gazebo_tp={self._gz_cli is not None}, "
            f"gz_svc={self._gz_service_name or 'none'})"
        )

    def _init_gazebo_client(self) -> None:
        if not HAVE_GZ:
            return
        for name in GZ_SERVICE_NAMES:
            cli = self.create_client(SetEntityState, name)
            if cli.wait_for_service(timeout_sec=self._gz_wait):
                self._gz_cli = cli
                self._gz_service_name = name
                self.get_logger().info(f"using Gazebo service {name}")
                return
        self.get_logger().warn(
            f"set_entity_state not found (tried {GZ_SERVICE_NAMES}); "
            "object teleport disabled until gzserver is up"
        )

    def _on_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        world_pose: Optional[Tuple[float, float, float]] = None
        joint1: Optional[float] = None
        if raw.startswith("{"):
            try:
                data = json.loads(raw)
            except ValueError as exc:
                self._report("?", ok=False, reason=f"bad JSON: {exc}")
                return
            cmd = str(data.get("cmd", "")).upper()
            if "joint1" in data:
                try:
                    joint1 = float(data["joint1"])
                except (TypeError, ValueError):
                    pass
            if "x" in data and "y" in data:
                try:
                    world_pose = (
                        float(data["x"]),
                        float(data["y"]),
                        float(data.get("z", 0.1)),
                    )
                except (TypeError, ValueError) as exc:
                    self._report(cmd, ok=False, reason=f"bad pose: {exc}")
                    return
        else:
            cmd = raw.upper()

        if cmd not in ("PICK", "PLACE", "HOME"):
            self._report(cmd, ok=False, reason="unknown command")
            return
        self._pending_joint1 = joint1
        threading.Thread(
            target=self._run_sequence, args=(cmd, world_pose), daemon=True,
        ).start()

    def _run_sequence(
        self, cmd: str, world_pose: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        if not self._busy.acquire(blocking=False):
            self._report(cmd, ok=False, reason="arm busy")
            return
        try:
            if cmd == "PICK":
                ok = self._pick(world_pose)
            elif cmd == "PLACE":
                ok = self._place(world_pose)
            else:
                ok = self._send_joints(self.home, "home")
                self._next_object_idx = 0
                self._held_object = None
            self._report(cmd, ok=ok)
        except Exception as exc:
            self.get_logger().error(f"arm sequence {cmd} failed: {exc}")
            self._report(cmd, ok=False, reason=str(exc))
        finally:
            self._pending_joint1 = None
            self._busy.release()

    def _with_joint1(self, base: List[float], joint1: Optional[float]) -> List[float]:
        pose = list(base)
        if joint1 is not None:
            pose[0] = float(joint1)
        return pose

    def _move_chassis(self, speed_x: float, seconds: float, label: str) -> None:
        """Drive /cmd_vel at speed_x for seconds, then publish a stop."""
        self.get_logger().info(
            f"chassis {label}: {speed_x:+.3f} m/s for {seconds:.2f}s"
        )
        cmd = Twist()
        cmd.linear.x = float(speed_x)
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        # brake
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def _pick(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info("PICK started")
        if self._next_object_idx >= len(OBJECT_NAMES):
            self.get_logger().warn("no objects left to pick")
            return False
        name = OBJECT_NAMES[self._next_object_idx]
        j1 = self._pending_joint1
        self.get_logger().info(f"PICK target {name} pose={pose} joint1={j1}")

        reach = self._with_joint1(self.reach, j1)
        lower = self._with_joint1(self.lower, j1)
        carry = self._with_joint1(self.carry, j1)

        # Choreography mirrors the working course reference:
        #   open → reach → back ~3cm → lower → close → forward ~2cm → carry
        # The back step gives the wrist room to swing down without clipping
        # the cube; the forward step wedges the cube between the fingers.
        arm_ok = True
        arm_ok &= self._send_gripper(self.g_open, "open gripper")
        arm_ok &= self._send_joints(reach, "reach")
        self._move_chassis(-self.pick_back_v, self.pick_back_t, "back before grasp")
        arm_ok &= self._send_joints(lower, "lower")
        arm_ok &= self._send_gripper(self.g_close, "close gripper")
        self._move_chassis(self.pick_fwd_v, self.pick_fwd_t, "fwd after grasp")
        arm_ok &= self._send_joints(carry, "carry")
        if not arm_ok:
            self.get_logger().warn("arm_controller trajectory incomplete")

        self._next_object_idx += 1
        self._held_object = name
        tp_ok = self._teleport_object(name, pose)
        self._send_joints(self.home, "home after pick")
        return self._pick_place_ok(tp_ok, arm_ok)

    def _place(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info(f"PLACE started target={pose} held={self._held_object}")
        j1 = self._pending_joint1
        reach = self._with_joint1(self.reach, j1)
        lower = self._with_joint1(self.lower, j1)

        arm_ok = True
        arm_ok &= self._send_joints(reach, "reach over target")
        arm_ok &= self._send_joints(lower, "lower")
        arm_ok &= self._send_gripper(self.g_open, "release")
        arm_ok &= self._send_joints(self.home, "home")

        tp_ok = True
        if self._held_object and pose is not None:
            x, y, z = pose
            off = -0.04 if self._held_object == OBJECT_NAMES[0] else 0.04
            tp_ok = self._teleport(self._held_object, x + off, y, max(z, 0.10))
            if tp_ok:
                self.get_logger().info(
                    f"teleport {self._held_object} -> "
                    f"({x + off:.2f}, {y:.2f}, {max(z, 0.10):.2f})"
                )
            self._held_object = None
        return self._pick_place_ok(tp_ok, arm_ok)

    def _teleport_object(
        self, name: str, pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        if self._gz_cli is None:
            self._init_gazebo_client()
        tp_ok = True
        if pose is not None:
            ox, oy, oz = pose
            side = -0.03 if name == OBJECT_NAMES[0] else 0.03
            tp_ok &= self._teleport(name, ox + side, oy, oz)
            time.sleep(0.2)
        tp_ok &= self._teleport(name, 0.0, 0.0, STOW_Z)
        if tp_ok:
            self.get_logger().info(f"carrying {name} (teleport to stow ok)")
        else:
            self.get_logger().error(f"PICK teleport failed for {name}")
        return tp_ok

    def _pick_place_ok(self, tp_ok: bool, arm_ok: bool) -> bool:
        if self._gz_cli is not None:
            return tp_ok
        return arm_ok

    def _teleport(self, name: str, x: float, y: float, z: float) -> bool:
        if self._gz_cli is None:
            if self.require_tp:
                self.get_logger().warn("set_entity_state unavailable — retrying")
                self._init_gazebo_client()
            if self._gz_cli is None:
                return not self.require_tp
        if not self._gz_cli.service_is_ready():
            if not self._gz_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f"{self._gz_service_name} not ready for {name}"
                )
                return False
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        future = self._gz_cli.call_async(req)
        ok = self._await_future(future, timeout=5.0, label=f"tp {name}")
        if ok:
            self.get_logger().info(
                f"Gazebo teleport {name} -> ({x:.2f}, {y:.2f}, {z:.2f})"
            )
        return ok

    def _send_joints(
        self,
        positions: List[float],
        label: str,
        path_time: Optional[float] = None,
    ) -> bool:
        t_move = self.step_wait if path_time is None else float(path_time)
        self.get_logger().info(f"joints {label}: {positions} ({t_move:.1f}s)")
        if HAVE_OM and self._joint_cli and self._joint_cli.wait_for_service(timeout_sec=1.0):
            req = SetJointPosition.Request()
            req.planning_group = "arm"
            req.joint_position.joint_name = list(JOINT_NAMES)
            req.joint_position.position = list(positions)
            req.path_time = t_move
            return self._await_future(
                self._joint_cli.call_async(req), t_move + 2.0, label
            )
        if self._arm_action and self._arm_action.wait_for_server(timeout_sec=1.0):
            traj = JointTrajectory()
            traj.joint_names = list(JOINT_NAMES)
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            sec = int(t_move)
            nsec = int((t_move - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points = [pt]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send_future = self._arm_action.send_goal_async(goal)
            if not self._await_future(send_future, 5.0, label):
                return False
            handle = send_future.result()
            if handle is None or not handle.accepted:
                return False
            return self._await_future(
                handle.get_result_async(), t_move + 2.0, label
            )
        time.sleep(t_move)
        return True

    def _send_gripper(self, position: float, label: str) -> bool:
        self.get_logger().info(f"gripper {label}: {position:.3f}")
        if HAVE_OM and self._tool_cli and self._tool_cli.wait_for_service(timeout_sec=1.0):
            req = SetJointPosition.Request()
            req.planning_group = "gripper"
            req.joint_position.joint_name = ["gripper"]
            req.joint_position.position = [position]
            req.path_time = self.step_wait
            return self._await_future(
                self._tool_cli.call_async(req), self.step_wait + 2.0, label
            )
        if self._gripper_action and self._gripper_action.wait_for_server(timeout_sec=1.0):
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = 5.0
            send_future = self._gripper_action.send_goal_async(goal)
            if not self._await_future(send_future, 5.0, label):
                return False
            handle = send_future.result()
            if handle is None or not handle.accepted:
                return False
            return self._await_future(
                handle.get_result_async(), self.step_wait + 2.0, label
            )
        time.sleep(self.step_wait)
        return True

    def _await_future(self, future, timeout: float, label: str) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if future.done():
                time.sleep(0.3)
                return True
            time.sleep(0.05)
        self.get_logger().warn(f"timeout: {label}")
        return False

    def _report(self, cmd: str, ok: bool, reason: str = "") -> None:
        self.get_logger().info(
            f"arm_status {cmd} ok={ok}" + (f" ({reason})" if reason else "")
        )
        self.status_pub.publish(String(data=json.dumps({
            "cmd": cmd, "ok": ok, "reason": reason,
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
