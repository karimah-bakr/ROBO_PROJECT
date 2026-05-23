#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteEntity, SpawnEntity

ARM_POSITIONS = {
    "home":  [0.0,  0.0,  0.0,  0.0],
    "reach": [0.0,  0.6, -0.2, -0.4],
    "grasp": [0.0,  0.9, -0.3, -0.5],
    "carry": [0.0,  0.0,  0.0,  0.0],
    "place": [0.0,  0.6, -0.2, -0.4],
}

GRIPPER_OPEN   =  0.019
GRIPPER_CLOSED = -0.019

OBJ_XML = {
    "object_1": """<?xml version="1.0"?>
<sdf version="1.6">
<model name="object_1">
  <static>false</static>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
      <material><ambient>1 0 0 1</ambient></material>
    </visual>
  </link>
</model>
</sdf>""",
    "object_2": """<?xml version="1.0"?>
<sdf version="1.6">
<model name="object_2">
  <static>false</static>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
      <material><ambient>0 0 1 1</ambient></material>
    </visual>
  </link>
</model>
</sdf>"""
}

class ArmController:
    def __init__(self, node: Node):
        self.node = node
        self.arm_pub = node.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self._gripper_client = ActionClient(
            node, GripperCommand, '/gripper_controller/gripper_cmd')
        self._gripper_client.wait_for_server(timeout_sec=5.0)
        self._delete_client = node.create_client(DeleteEntity, '/delete_entity')
        self._spawn_client  = node.create_client(SpawnEntity, '/spawn_entity')
        self._carried_object = None
        self.node.get_logger().info('✅ gripper جاهز')

    def _brake(self):
        cmd = Twist()
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)

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

    def _delete_object(self, name):
        if not self._delete_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('delete_entity غير متاح')
            return
        req = DeleteEntity.Request()
        req.name = name
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        time.sleep(1.0)
        self.node.get_logger().info(f'🗑️ حذف {name}')

    def _spawn_object(self, name, x, y, z=0.05):
        if not self._spawn_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('spawn_entity غير متاح')
            return
        from geometry_msgs.msg import Pose
        req = SpawnEntity.Request()
        req.name = name
        req.xml  = OBJ_XML[name]
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        req.initial_pose.orientation.w = 1.0
        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.node.get_logger().info(f'✨ إنشاء {name} في ({x:.2f}, {y:.2f})')

    def pick_object(self, object_name, robot_x, robot_y):
        self.node.get_logger().info(f'═══ بدء الإمساك {object_name} ═══')
        self._brake()
        self.move_gripper(GRIPPER_OPEN)
        self._spin_seconds(1.0)
        self.move_arm("reach", 3)
        self._spin_seconds(3.0)
        self.move_gripper(GRIPPER_CLOSED)
        self._spin_seconds(1.0)
        self._delete_object(object_name)
        self._carried_object = object_name
        self.move_arm("carry", 3)
        self._spin_seconds(3.0)
        self.node.get_logger().info(f'═══ تم الإمساك {object_name} ═══')

    def place_object(self, target_x, target_y):
        self.node.get_logger().info('═══ بدء الوضع ═══')
        self._brake()
        self.move_arm("reach", 3)
        self._spin_seconds(3.0)
        self.move_arm("place", 3)
        self._spin_seconds(3.0)
        self.move_gripper(GRIPPER_OPEN)
        self._spin_seconds(1.0)
        if self._carried_object:
            self.node.get_logger().info(f'spawning {self._carried_object} at ({target_x:.2f}, {target_y:.2f})')
            self._spawn_object(self._carried_object, target_x, target_y, z=0.05)
            self._carried_object = None
        self.move_arm("carry", 3)
        self._spin_seconds(3.0)
        self.move_arm("home", 3)
        self._spin_seconds(3.0)
        self.node.get_logger().info('═══ تم الوضع ═══')

    def go_home(self):
        self.move_arm("home", 3)
        self._spin_seconds(3.0)
