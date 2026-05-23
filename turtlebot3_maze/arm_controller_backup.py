#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist

ARM_POSITIONS = {
    "home":  [0.0,  0.0,  0.0,  0.0],
    "reach": [0.0,  0.6, -0.2, -0.4],
    "grasp": [0.0,  0.9, -0.3, -0.5],
    "carry": [0.0,  0.0,  0.0,  0.0],
    "place": [0.0,  0.6, -0.2, -0.4],
}

GRIPPER_OPEN   =  0.019
GRIPPER_CLOSED = -0.019

class ArmController:
    def __init__(self, node: Node):
        self.node = node
        self.arm_pub = node.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self._gripper_client = ActionClient(
            node, GripperCommand, '/gripper_controller/gripper_cmd')
        self._gripper_client.wait_for_server(timeout_sec=5.0)
        self.node.get_logger().info('✅ gripper جاهز')

    def _brake(self):
        cmd = Twist()
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _move_forward(self, speed=0.03, seconds=1.0):
        cmd = Twist()
        cmd.linear.x = speed
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < seconds * 1e9:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._brake()

    def _move_backward(self, speed=0.03, seconds=1.0):
        cmd = Twist()
        cmd.linear.x = -speed
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < seconds * 1e9:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._brake()

    def _turn_right(self, seconds=1.5):
        cmd = Twist()
        cmd.angular.z = -0.4
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < seconds * 1e9:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._brake()

    def _turn_left(self, seconds=1.5):
        cmd = Twist()
        cmd.angular.z = 0.4
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < seconds * 1e9:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self._brake()

    def move_arm(self, angles, duration_sec=3):
        if isinstance(angles, str):
            angles = ARM_POSITIONS[angles]
        msg = JointTrajectory()
        msg.joint_names = ['joint1','joint2','joint3','joint4']
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in angles]
        pt.time_from_start = Duration(sec=duration_sec)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def move_gripper(self, position, max_effort=10.0):
        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = max_effort
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

    def _spin_seconds(self, seconds):
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start).nanoseconds < seconds * 1e9:
            self._brake()

    def pick_object(self, object_name=None):
        self.node.get_logger().info('═══ بدء الإمساك ═══')
        self._brake()
        self.move_gripper(GRIPPER_OPEN)
        self._spin_seconds(2.0)
        self.move_arm("reach", 4)
        self._spin_seconds(5.0)
        self._move_backward(0.02, 1.5)
        self.move_arm("grasp", 4)
        self._spin_seconds(5.0)
        self.move_gripper(GRIPPER_CLOSED)
        self._spin_seconds(4.0)
        self._move_forward(0.02, 1.0)
        self.move_arm("reach", 5)
        self._spin_seconds(6.0)
        self.move_arm("carry", 5)
        self._spin_seconds(6.0)
        self.node.get_logger().info('═══ تم الإمساك ═══')

    def place_object(self, target_x=None, target_y=None):
        self.node.get_logger().info('═══ بدء الوضع ═══')
        self._brake()
        self.move_arm("reach", 4)
        self._spin_seconds(5.0)
        self.move_arm("place", 4)
        self._spin_seconds(5.0)
        self.move_gripper(GRIPPER_OPEN)
        self._spin_seconds(3.0)
        self.move_arm("carry", 4)
        self._spin_seconds(5.0)
        self.move_arm("home", 4)
        self._spin_seconds(5.0)
        self.node.get_logger().info('═══ تم الوضع ═══')

    def go_home(self):
        self.move_arm("home", 3)
        self._spin_seconds(3.0)
