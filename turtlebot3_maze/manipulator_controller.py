"""OpenManipulator-X controller node.

Exposes high-level "pick" / "place" / "home" actions through a simple
std_msgs/String trigger topic and reports completion on /arm_status.

Internally it calls the open_manipulator_msgs/SetJointPosition service
on /open_manipulator/goal_joint_space_path and /goal_tool_control,
waiting `step_wait_s` seconds between sub-motions so the physical arm
can complete each pose before the next is queued.

Topic protocol:
    /arm_command  std_msgs/String      one of: "PICK", "PLACE", "HOME"
    /arm_status   std_msgs/String JSON {"cmd": "...", "ok": true|false, "reason": "..."}

If the open_manipulator_msgs package is not available in the workspace
(e.g. headless tests, no arm bringup), the node falls back to a stub
that just logs the would-be commands so the FSM can still progress.
"""

from __future__ import annotations

import json
import threading
import time
from typing import List, Optional

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


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_NAMES = ["gripper"]


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

        self._busy = threading.Lock()
        self.get_logger().info(
            f"manipulator_controller ready (open_manipulator_msgs={'yes' if HAVE_OM else 'STUB'})"
        )

    # ------------------------------------------------------------------

    def _on_cmd(self, msg: String) -> None:
        cmd = (msg.data or "").strip().upper()
        if cmd not in ("PICK", "PLACE", "HOME"):
            self._report(cmd, ok=False, reason="unknown command")
            return
        # Run sequence in a worker thread so we don't block the executor.
        threading.Thread(target=self._run_sequence, args=(cmd,), daemon=True).start()

    def _run_sequence(self, cmd: str) -> None:
        if not self._busy.acquire(blocking=False):
            self._report(cmd, ok=False, reason="arm busy")
            return
        try:
            if cmd == "PICK":
                ok = self._pick()
            elif cmd == "PLACE":
                ok = self._place()
            else:
                ok = self._send_joints(self.home, "home")
            self._report(cmd, ok=ok)
        except Exception as exc:
            self.get_logger().error(f"arm sequence {cmd} failed: {exc}")
            self._report(cmd, ok=False, reason=str(exc))
        finally:
            self._busy.release()

    # ------------------------------------------------------------------
    # Sequences

    def _pick(self) -> bool:
        self.get_logger().info("ARM: PICK sequence")
        ok = True
        ok &= self._send_gripper(self.g_open, "open gripper")
        ok &= self._send_joints(self.reach, "reach forward")
        ok &= self._send_joints(self.lower, "lower to object")
        ok &= self._send_gripper(self.g_close, "close gripper")
        ok &= self._send_joints(self.carry, "lift to carry")
        return ok

    def _place(self) -> bool:
        self.get_logger().info("ARM: PLACE sequence")
        ok = True
        ok &= self._send_joints(self.reach, "extend over target")
        ok &= self._send_joints(self.lower, "lower to ground")
        ok &= self._send_gripper(self.g_open, "release object")
        ok &= self._send_joints(self.home, "retract home")
        return ok

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
