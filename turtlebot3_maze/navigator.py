#!/usr/bin/env python3
import math
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

CELL = 0.7
N    = 7

h = [
    [1,1,1,1,1,1,1],
    [0,0,1,0,1,0,0],
    [1,0,1,0,1,0,1],
    [1,0,0,0,1,0,1],
    [1,1,1,0,1,1,1],
    [1,0,0,0,0,0,1],
    [1,0,1,1,1,1,1],
    [0,0,1,1,1,1,1],
]

v = [
    [1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0],
    [1,0,1,0,1,0,1],
    [1,0,1,0,1,0,1],
    [1,1,1,0,1,1,1],
    [1,0,1,0,0,0,1],
    [1,0,1,1,1,0,1],
    [1,1,1,1,1,1,1],
]

def has_wall(row, col, direction):
    if direction == 'N': return bool(h[row+1][col])
    if direction == 'S': return bool(h[row][col])
    if direction == 'E': return bool(v[col+1][row])
    if direction == 'W': return bool(v[col][row])
    return True

def get_neighbours(row, col):
    nbrs = []
    if row < N-1 and not has_wall(row, col, 'N'): nbrs.append((row+1, col))
    if row > 0   and not has_wall(row, col, 'S'): nbrs.append((row-1, col))
    if col < N-1 and not has_wall(row, col, 'E'): nbrs.append((row, col+1))
    if col > 0   and not has_wall(row, col, 'W'): nbrs.append((row, col-1))
    return nbrs

def bfs(start, goal):
    if start == goal: return [start]
    queue = deque([[start]])
    visited = {start}
    while queue:
        path = queue.popleft()
        for nb in get_neighbours(*path[-1]):
            if nb == goal: return path + [nb]
            if nb not in visited:
                visited.add(nb)
                queue.append(path + [nb])
    return []

def cell_center(row, col):
    return col * CELL + CELL/2, row * CELL + CELL/2

def world_to_cell(x, y):
    col = max(0, min(N-1, int(x / CELL)))
    row = max(0, min(N-1, int(y / CELL)))
    return row, col

SPEED      = 0.12
TURN_SPEED = 0.40
GOAL_TOL   = 0.18
ANGLE_TOL  = 0.05

class Navigator:
    def __init__(self, node):
        self.node = node
        self.pub  = node.create_publisher(Twist, '/cmd_vel', 10)
        node.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        node.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.front = 999.0
        self._path = []
        self._path_idx = 0
        self._state = 'IDLE'
        self._target_yaw = 0.0
        self._target_x = self._target_y = 0.0

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _scan_cb(self, msg):
        r, n = msg.ranges, len(msg.ranges)
        def s(i):
            v = r[i % n]
            return v if math.isfinite(v) and v > 0.01 else 999.0
        self.front = min(s(i) for i in range(-15, 15))

    @property
    def position(self):
        return self.x, self.y

    @property
    def current_cell(self):
        return world_to_cell(self.x, self.y)

    def go_to_cell(self, goal_row, goal_col):
        start = world_to_cell(self.x, self.y)
        goal  = (goal_row, goal_col)
        path  = bfs(start, goal)
        if path:
            self._path = path
            self._path_idx = 0
            self._state = 'IDLE'
            self.node.get_logger().info(f'BFS: {len(path)} خلية {start}→{goal}')
        else:
            self.node.get_logger().warn(f'BFS: لا يوجد مسار!')

    def go_to_xy(self, x, y):
        r, c = world_to_cell(x, y)
        self.go_to_cell(r, c)

    def stop(self):
        self.pub.publish(Twist())
        self._state = 'IDLE'

    def is_done(self):
        return self._state == 'IDLE' and self._path_idx >= len(self._path)

    def _angle_diff(self, target, current):
        e = target - current
        while e >  math.pi: e -= 2*math.pi
        while e < -math.pi: e += 2*math.pi
        return e

    def step(self, goal_x=None, goal_y=None):
        if goal_x is not None and not self._path:
            self.go_to_xy(goal_x, goal_y)
        if not self._path or self._path_idx >= len(self._path):
            self.stop()
            return True

        target_cell = self._path[self._path_idx]
        tx, ty = cell_center(*target_cell)

        if self._state == 'TURNING':
            err = self._angle_diff(self._target_yaw, self.yaw)
            cmd = Twist()
            if abs(err) > ANGLE_TOL:
                cmd.angular.z = TURN_SPEED * (1.0 if err > 0 else -1.0)
                if abs(err) < 0.3: cmd.angular.z *= 0.5
                self.pub.publish(cmd)
            else:
                self.stop()
                self._state = 'MOVING'
                self._target_x, self._target_y = tx, ty
            return False

        if self._state == 'MOVING':
            dist = math.hypot(tx - self.x, ty - self.y)
            if dist < GOAL_TOL:
                self.stop()
                self._path_idx += 1
                self._state = 'IDLE'
                self.node.get_logger().info(f'✅ خلية {target_cell} | باقي {len(self._path)-self._path_idx}')
                return self._path_idx >= len(self._path)
            ang = math.atan2(ty - self.y, tx - self.x)
            err = self._angle_diff(ang, self.yaw)
            if abs(err) > 0.3:
                self.stop()
                self._target_yaw = ang
                self._state = 'TURNING'
                return False
            cmd = Twist()
            cmd.linear.x  = SPEED
            cmd.angular.z = max(-0.3, min(0.3, err * 2.0))
            self.pub.publish(cmd)
            return False

        if self._state == 'IDLE':
            if self._path_idx >= len(self._path):
                return True
            tx, ty = cell_center(*self._path[self._path_idx])
            target_yaw = math.atan2(ty - self.y, tx - self.x)
            err = self._angle_diff(target_yaw, self.yaw)
            if abs(err) > ANGLE_TOL:
                self._target_yaw = target_yaw
                self._state = 'TURNING'
            else:
                self._target_x, self._target_y = tx, ty
                self._state = 'MOVING'
        return False

def main():
    rclpy.init()
    node = rclpy.create_node('navigator')
    nav  = Navigator(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
