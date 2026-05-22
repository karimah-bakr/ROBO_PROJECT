"""OpenManipulator-X controller node.

Exposes high-level "pick" / "place" / "home" actions through a simple
trigger topic and reports completion on /arm_status.

Topic protocol:
    /arm_command  std_msgs/String      Plain command ("PICK"/"PLACE"/"HOME") OR
                                       JSON {"cmd": "PICK",  "x": 0.25, "y": 2.25, "z": 0.1}
                                       JSON {"cmd": "PLACE", "x": 3.25, "y": 0.25, "z": 0.1}
                                       x/y/z = world pose of the object cell (pick) or
                                       target cell centre (place).
    /arm_status   std_msgs/String JSON {"cmd": "...", "ok": true|false, "reason": "..."}

Internally:
    - The arm motions go through open_manipulator_msgs/SetJointPosition if the
      service is up (real hardware / full bringup). Otherwise it's a sleep stub
      so the FSM can still progress in headless tests.
    - To make the two-object delivery actually visible in Gazebo, the node
      teleports the held SDF model via gazebo_msgs/SetEntityState: object_1 on
      the first PICK and object_2 on the second, "stowed" off-screen while
      carried and dropped at the place pose. Without gazebo_msgs the teleport
      step is silently skipped.
"""

from __future__ import annotations

import json
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Try to import the real service type; fall back to None if absent.
try:  # pragma: no cover  -- environment dependent
    from open_manipulator_msgs.srv import SetJointPosition  # type: ignore
    HAVE_OM = True
except Exception:  # pragma: no cover
    SetJointPosition = None  # type: ignore
    HAVE_OM = False

try:  # pragma: no cover  -- environment dependent
    from gazebo_msgs.srv import SetEntityState  # type: ignore
    HAVE_GZ = True
except Exception:  # pragma: no cover
    SetEntityState = None  # type: ignore
    HAVE_GZ = False


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_NAMES = ["gripper"]
OBJECT_NAMES = ("object_1", "object_2")
STOW_Z = -2.0  # park height while the object is being carried


class ManipulatorControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_controller")

        self.declare_parameter("home_joints",  [0.0, -1.05,  0.35,  0.70])
        self.declare_parameter("reach_joints", [0.0,  0.20,  0.30, -0.50])
        self.declare_parameter("lower_joints", [0.0,  0.60,  0.10, -0.70])
        self.declare_parameter("carry_joints", [0.0, -0.90,  0.30,  0.60])
        self.declare_parameter("gripper_open",  0.044)
        self.declare_parameter("gripper_close", 0.010)
        self.declare_parameter("step_wait_s",   2.0)

        self.home     = list(self.get_parameter("home_joints").value)
        self.reach    = list(self.get_parameter("reach_joints").value)
        self.lower    = list(self.get_parameter("lower_joints").value)
        self.carry    = list(self.get_parameter("carry_joints").value)
        self.g_open   = float(self.get_parameter("gripper_open").value)
        self.g_close  = float(self.get_parameter("gripper_close").value)
        self.step_wait = float(self.get_parameter("step_wait_s").value)

        self.cmd_sub = self.create_subscription(String, "/arm_command", self._on_cmd, 5)
        self.status_pub = self.create_publisher(String, "/arm_status", 10)

        # Service clients (may be unavailable; we don't block startup on them).
        self._joint_cli = None
        self._tool_cli = None
        if HAVE_OM:
            self._joint_cli = self.create_client(
                SetJointPosition, "/open_manipulator/goal_joint_space_path"
            )
            self._tool_cli = self.create_client(
                SetJointPosition, "/open_manipulator/goal_tool_control"
            )

        # Optional Gazebo teleport client (for the two-object visual handoff).
        self._gz_cli = None
        if HAVE_GZ:
            self._gz_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        # Two objects: object_1 first, object_2 second. The counter advances
        # on every PICK and is reset by HOME for re-runs.
        self._next_object_idx = 0
        self._held_object: Optional[str] = None

        self._busy = threading.Lock()
        self.get_logger().info(
            "manipulator_controller ready "
            f"(open_manipulator_msgs={'yes' if HAVE_OM else 'STUB'}, "
            f"gazebo_set_entity_state={'yes' if HAVE_GZ else 'no'})"
        )

    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Sequences

    def _pick(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info(f"ARM: PICK sequence (object pose={pose})")
        ok = True
        ok &= self._send_gripper(self.g_open, "open gripper")
        ok &= self._send_joints(self.reach, "reach forward")
        ok &= self._send_joints(self.lower, "lower to object")
        ok &= self._send_gripper(self.g_close, "close gripper")
        ok &= self._send_joints(self.carry, "lift to carry")
        if self._next_object_idx >= len(OBJECT_NAMES):
            self.get_logger().warn("PICK requested but no objects left to grab")
            return ok
        name = OBJECT_NAMES[self._next_object_idx]
        self._next_object_idx += 1
        self._held_object = name
        # In Gazebo (burger has no arm mesh): teleport the cube off the floor.
        if pose is not None:
            ox, oy, oz = pose
            side = -0.03 if name == OBJECT_NAMES[0] else 0.03
            self._teleport(name, x=ox + side, y=oy, z=oz)
            time.sleep(0.3)
        self._teleport(name, x=0.0, y=0.0, z=STOW_Z)
        self.get_logger().info(f"carrying {name} (stowed below world)")
        return ok

    def _place(self, pose: Optional[Tuple[float, float, float]]) -> bool:
        self.get_logger().info(f"ARM: PLACE sequence (target pose={pose})")
        ok = True
        ok &= self._send_joints(self.reach, "extend over target")
        ok &= self._send_joints(self.lower, "lower to ground")
        ok &= self._send_gripper(self.g_open, "release object")
        ok &= self._send_joints(self.home, "retract home")
        if self._held_object is None:
            self.get_logger().warn("PLACE: nothing held")
            return ok
        if pose is None:
            self.get_logger().warn("PLACE: no target pose; object stays stowed")
            return ok
        x, y, z = pose
        offset = -0.04 if self._held_object == OBJECT_NAMES[0] else 0.04
        drop_x, drop_y, drop_z = x + offset, y, max(z, 0.10)
        ok &= self._teleport(self._held_object, x=drop_x, y=drop_y, z=drop_z)
        self.get_logger().info(
            f"dropped {self._held_object} at target cell ({drop_x:.2f}, {drop_y:.2f})"
        )
        self._held_object = None
        return ok

    # ------------------------------------------------------------------
    # Gazebo teleport

    def _teleport(self, name: str, x: float, y: float, z: float) -> bool:
        if self._gz_cli is None:
            self.get_logger().warn(
                f"gazebo_msgs unavailable — cannot move {name} (install ros-humble-gazebo-ros-pkgs)"
            )
            return True  # allow FSM to continue in stub mode
        if not self._gz_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("set_entity_state not ready; skipping teleport")
            return False
        req = SetEntityState.Request()
        try:
            req.state.name = name
            req.state.pose.position.x = float(x)
            req.state.pose.position.y = float(y)
            req.state.pose.position.z = float(z)
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame = "world"
        except Exception as exc:
            self.get_logger().warn(f"could not fill teleport request: {exc}")
            return False
        future = self._gz_cli.call_async(req)
        return self._await(future, timeout=5.0, label=f"teleport {name}")

    # ------------------------------------------------------------------
    # Low-level helpers

    def _send_joints(self, positions: List[float], label: str) -> bool:
        self.get_logger().info(f"ARM joint -> {label}: {positions}")
        if not HAVE_OM or self._joint_cli is None:
            time.sleep(self.step_wait)
            return True
        if not self._joint_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"joint service not ready (skipping {label})")
            time.sleep(self.step_wait)
            return False
        req = SetJointPosition.Request()
        try:
            # The exact field set depends on the open_manipulator_msgs version.
            # The common layout has planning_group, joint_position, path_time.
            req.planning_group = "arm"
            req.joint_position.joint_name = list(JOINT_NAMES)
            req.joint_position.position = list(positions)
            req.path_time = self.step_wait
        except Exception as exc:
            self.get_logger().warn(f"could not fill joint request: {exc}")
            return False
        future = self._joint_cli.call_async(req)
        return self._await(future, timeout=self.step_wait + 2.0, label=label)

    def _send_gripper(self, position: float, label: str) -> bool:
        self.get_logger().info(f"ARM tool -> {label}: {position:.3f}")
        if not HAVE_OM or self._tool_cli is None:
            time.sleep(self.step_wait)
            return True
        if not self._tool_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"tool service not ready (skipping {label})")
            time.sleep(self.step_wait)
            return False
        req = SetJointPosition.Request()
        try:
            req.planning_group = "gripper"
            req.joint_position.joint_name = list(GRIPPER_NAMES)
            req.joint_position.position = [position]
            req.path_time = self.step_wait
        except Exception as exc:
            self.get_logger().warn(f"could not fill tool request: {exc}")
            return False
        future = self._tool_cli.call_async(req)
        return self._await(future, timeout=self.step_wait + 2.0, label=label)

    def _await(self, future, timeout: float, label: str) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if future.done():
                # We do not inspect the response: many bringup configs return
                # a custom is_planned bool; treating "no exception" as success
                # is sufficient for the FSM. Pause one step before returning.
                time.sleep(self.step_wait)
                return True
            time.sleep(0.05)
        self.get_logger().warn(f"arm step '{label}' timed out")
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
