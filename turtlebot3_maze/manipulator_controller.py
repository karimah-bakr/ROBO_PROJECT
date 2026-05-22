"""OpenManipulator-X controller for TurtleBot3 manipulation (Gazebo / hardware).

/arm_command  JSON or plain text:
    {"cmd": "PICK",  "x": 0.25, "y": 2.25, "z": 0.1}
    {"cmd": "PLACE", "x": 3.25, "y": 0.25, "z": 0.1}
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


class ManipulatorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_controller")

        self.declare_parameter("home_joints",  [0.0, -1.05,  0.35,  0.70])
        self.declare_parameter("reach_joints", [0.0,  0.20,  0.30, -0.50])
        self.declare_parameter("lower_joints", [0.0,  0.60,  0.10, -0.70])
        self.declare_parameter("carry_joints", [0.0, -0.90,  0.30,  0.60])
        self.declare_parameter("gripper_open",  0.044)
        self.declare_parameter("gripper_close", 0.010)
        self.declare_parameter("step_wait_s",   1.5)

        self.home = list(self.get_parameter("home_joints").value)
        self.reach = list(self.get_parameter("reach_joints").value)
        self.lower = list(self.get_parameter("lower_joints").value)
        self.carry = list(self.get_parameter("carry_joints").value)
        self.g_open = float(self.get_parameter("gripper_open").value)
        self.g_close = float(self.get_parameter("gripper_close").value)
        self.step_wait = float(self.get_parameter("step_wait_s").value)

        self.cmd_sub = self.create_subscription(String, "/arm_command", self._on_cmd, 5)
        self.status_pub = self.create_publisher(String, "/arm_status", 10)

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
        if HAVE_GZ:
            self._gz_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self._next_object_idx = 0
        self._held_object: Optional[str] = None
        self._busy = threading.Lock()

        self.get_logger().info(
            "manipulator_controller ready "
            f"(open_manipulator={HAVE_OM}, ros2_control={HAVE_RC}, gazebo_tp={HAVE_GZ})"
        )

    def _on_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        world_pose: Optional[Tuple[float, float, float]] = None
        if raw.startswith("{"):
            try:
                data = json.loads(raw)
            except ValueError as exc:
                self._report("?", ok=False, reason=f"bad JSON: {exc}")
                return
            cmd = str(data.get("cmd", "")).upper()
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
            self._busy.release()

    def _pick(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info(f"ARM: PICK (object pose={pose})")
        ok = True
        ok &= self._send_gripper(self.g_open, "open gripper")
        ok &= self._send_joints(self.reach, "reach")
        ok &= self._send_joints(self.lower, "lower")
        ok &= self._send_gripper(self.g_close, "close gripper")
        ok &= self._send_joints(self.carry, "carry")
        if self._next_object_idx >= len(OBJECT_NAMES):
            self.get_logger().warn("no objects left to pick")
            return ok
        name = OBJECT_NAMES[self._next_object_idx]
        self._next_object_idx += 1
        self._held_object = name
        if pose is not None:
            ox, oy, oz = pose
            side = -0.03 if name == OBJECT_NAMES[0] else 0.03
            self._teleport(name, ox + side, oy, oz)
            time.sleep(0.2)
        self._teleport(name, 0.0, 0.0, STOW_Z)
        self.get_logger().info(f"carrying {name}")
        return ok

    def _place(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info(f"ARM: PLACE (target={pose})")
        ok = True
        ok &= self._send_joints(self.reach, "reach over target")
        ok &= self._send_joints(self.lower, "lower")
        ok &= self._send_gripper(self.g_open, "release")
        ok &= self._send_joints(self.home, "home")
        if self._held_object and pose is not None:
            x, y, z = pose
            off = -0.04 if self._held_object == OBJECT_NAMES[0] else 0.04
            self._teleport(self._held_object, x + off, y, max(z, 0.10))
            self.get_logger().info(
                f"dropped {self._held_object} at ({x + off:.2f}, {y:.2f})"
            )
            self._held_object = None
        return ok

    def _teleport(self, name: str, x: float, y: float, z: float) -> bool:
        if self._gz_cli is None:
            return True
        if not self._gz_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("set_entity_state unavailable")
            return False
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        future = self._gz_cli.call_async(req)
        return self._await_future(future, timeout=5.0, label=f"tp {name}")

    def _send_joints(self, positions: List[float], label: str) -> bool:
        self.get_logger().info(f"joints {label}: {positions}")
        if HAVE_OM and self._joint_cli and self._joint_cli.wait_for_service(timeout_sec=1.0):
            req = SetJointPosition.Request()
            req.planning_group = "arm"
            req.joint_position.joint_name = list(JOINT_NAMES)
            req.joint_position.position = list(positions)
            req.path_time = self.step_wait
            return self._await_future(
                self._joint_cli.call_async(req), self.step_wait + 2.0, label
            )
        if self._arm_action and self._arm_action.wait_for_server(timeout_sec=1.0):
            traj = JointTrajectory()
            traj.joint_names = list(JOINT_NAMES)
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.time_from_start = Duration(sec=int(self.step_wait), nanosec=0)
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
                handle.get_result_async(), self.step_wait + 2.0, label
            )
        time.sleep(self.step_wait)
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
