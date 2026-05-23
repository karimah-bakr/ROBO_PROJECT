#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot3_maze.navigator import Navigator
from turtlebot3_maze.arm_controller import ArmController
from turtlebot3_maze.object_detector import ObjectDetector
from gazebo_msgs.srv import SpawnEntity
import time

START_CELL  = (0, 2)
TARGET_CELL = (0, 6)
OBJECT_CELL = (4, 0)

TARGET_X = (7 - 0.5) * 0.7
TARGET_Y = (1 - 0.5) * 0.7

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.nav      = Navigator(self)
        self.arm      = ArmController(self)
        self.detector = ObjectDetector(self)
        self._spawn_objects()
        self.state     = 'GOTO_OBJ'
        self._act_done = False
        r, c = OBJECT_CELL
        self.nav.go_to_cell(r, c)
        self.get_logger().info('═══ المهمة بدأت ═══')
        self.create_timer(0.05, self._loop)

    def _spawn_objects(self):
        client = self.create_client(SpawnEntity, '/spawn_entity')
        client.wait_for_service(timeout_sec=5.0)

        base_x = (1 - 0.5) * 0.7 - 0.05
        base_y = (5 - 0.5) * 0.7

        for name, y_off, color in [
            ("object_1", +0.07, "1 0 0 1"),
            ("object_2", -0.07, "0 0 1 1"),
        ]:
            sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
<model name="{name}">
  <static>false</static>
  <pose>{base_x:.3f} {base_y + y_off:.3f} 0.1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
      <material><ambient>{color}</ambient></material>
    </visual>
  </link>
</model>
</sdf>"""

            req = SpawnEntity.Request()
            req.name = name
            req.xml  = sdf
            client.call_async(req)
            time.sleep(0.5)

        self.get_logger().info('تم إنشاء الأوبجكتين ✅')

    def _loop(self):
        s = self.state

        if s == 'GOTO_OBJ':
            if self.nav.step():
                self.get_logger().info('وصلنا للجسم 1 ✅')
                self.state = 'PICK'

        elif s == 'PICK':
            self.nav.stop()
            self.arm.pick_object("object_1", self.nav.x, self.nav.y)
            self.get_logger().info('تم إمساك الجسم 1 ✅')
            self.nav.go_to_cell(*TARGET_CELL)
            self.state = 'GOTO_TGT'

        elif s == 'GOTO_TGT':
            if self.nav.step():
                self.get_logger().info('وصلنا للهدف ✅')
                self.state = 'PLACE'

        elif s == 'PLACE':
            self.nav.stop()
            self.arm.place_object(TARGET_X, TARGET_Y)
            self.get_logger().info('تم وضع الجسم 1 ✅')
            self.nav.go_to_cell(*OBJECT_CELL)
            self.state = 'GOTO_OBJ2'

        elif s == 'GOTO_OBJ2':
            if self.nav.step():
                self.get_logger().info('وصلنا للجسم 2 ✅')
                self.state = 'PICK2'

        elif s == 'PICK2':
            self.nav.stop()
            self.arm.pick_object("object_2", self.nav.x, self.nav.y)
            self.get_logger().info('تم إمساك الجسم 2 ✅')
            self.nav.go_to_cell(*TARGET_CELL)
            self.state = 'GOTO_TGT2'

        elif s == 'GOTO_TGT2':
            if self.nav.step():
                self.get_logger().info('وصلنا للهدف ✅')
                self.state = 'PLACE2'

        elif s == 'PLACE2':
            self.nav.stop()
            self.arm.place_object(TARGET_X, TARGET_Y + 0.1)
            self.get_logger().info('تم وضع الجسم 2 ✅')
            self.nav.go_to_cell(*START_CELL)
            self.state = 'GOTO_HOME'

        elif s == 'GOTO_HOME':
            if self.nav.step():
                self.nav.stop()
                self.arm.go_home()
                self.state = 'DONE'
                self.get_logger().info('🏁 المهمة اكتملت!')

        elif s == 'DONE':
            self.nav.stop()

def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.nav.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
