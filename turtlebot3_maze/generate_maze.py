#!/usr/bin/env python3
import os

CELL = 0.7
WALL_T = 0.015
WALL_H = 0.2
N = 7

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

def box(name, x, y, sx, sy, sz=WALL_H):
    return f"""
<model name="{name}">
  <static>true</static>
  <pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
    </visual>
  </link>
</model>
"""

world = []

world.append("""<?xml version="1.0" ?>
<sdf version="1.6">
<world name="maze_world">
<include><uri>model://ground_plane</uri></include>
<include><uri>model://sun</uri></include>
""")

idx = 0

for y in range(N + 1):
    for x in range(N):
        if h[y][x]:
            cx = (x + 0.5) * CELL
            cy = y * CELL
            world.append(box(f"h_{idx}", cx, cy, CELL, WALL_T))
            idx += 1

for x in range(N + 1):
    for y in range(N):
        if v[x][y]:
            cx = x * CELL
            cy = (y + 0.5) * CELL
            world.append(box(f"v_{idx}", cx, cy, WALL_T, CELL))
            idx += 1

target_row, target_col = 1, 7
target_x = (target_col - 0.5) * CELL
target_y = (target_row - 0.5) * CELL

world.append(f"""
<model name="target_marker">
  <static>true</static>
  <pose>{target_x:.3f} {target_y:.3f} 0.01 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry><box><size>0.45 0.45 0.02</size></box></geometry>
      <material><ambient>0 1 0 0.5</ambient></material>
    </visual>
  </link>
</model>
""")

world.append("</world></sdf>")

path = os.path.expanduser(
    "~/turtlebot3_maze_ws/src/turtlebot3_maze/worlds/maze.world"
)

os.makedirs(os.path.dirname(path), exist_ok=True)

with open(path, "w") as f:
    f.write("".join(world))

print("[OK] Maze generated successfully")
