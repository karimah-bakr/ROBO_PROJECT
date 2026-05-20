#!/usr/bin/env python3
"""Generate worlds/maze_world.world from the official 7x7 maze spec."""
import os
from pathlib import Path

CELL = 0.5
WALL_T = 0.015
WALL_H = 0.2
N = 7

# المتاهة مع مخرج قريب من الأوبجكت
h = [
    [1, 1, 1, 1, 1, 1, 1],
    [0, 0, 1, 0, 1, 0, 0],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 1, 1],
    [0, 0, 1, 1, 1, 1, 1],  # مخرج قريب من الأوبجكت
]

v = [
    [1, 1, 1, 1, 1, 1, 1],
    [0, 1, 0, 1, 0, 1, 0],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 0, 1, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 1, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 1],
]


def model_box(name: str, x: float, y: float, sx: float, sy: float, sz: float = WALL_H) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def main() -> None:
    parts: list[str] = []
    parts.append(
        """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_world">
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
"""
    )

    idx = 0
    for y in range(N + 1):
        for x in range(N):
            if h[y][x]:
                if y == 0 and x in (0, 3, 6):
                    continue
                cx = (x + 0.5) * CELL
                cy = y * CELL
                parts.append(model_box(f"h_{idx}", cx, cy, CELL, WALL_T))
                idx += 1

    for x in range(N + 1):
        for y in range(N):
            if v[x][y]:
                cx = x * CELL
                cy = (y + 0.5) * CELL
                parts.append(model_box(f"v_{idx}", cx, cy, WALL_T, CELL))
                idx += 1

    row, col = 5, 1
    base_x = (col - 0.5) * CELL
    base_y = (row - 0.5) * CELL

    parts.append(
        f"""
    <model name="object_1">
      <static>false</static>
      <pose>{base_x - 0.03:.3f} {base_y:.3f} 0.1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.05</mass>
          <inertia><ixx>1.7e-4</ixx><iyy>1.7e-4</iyy><izz>5e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="col"><geometry><box><size>0.02 0.03 0.2</size></box></geometry></collision>
        <visual name="vis">
          <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
          <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
        </visual>
      </link>
    </model>
"""
    )
    parts.append(
        f"""
    <model name="object_2">
      <static>false</static>
      <pose>{base_x + 0.03:.3f} {base_y:.3f} 0.1 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.05</mass>
          <inertia><ixx>1.7e-4</ixx><iyy>1.7e-4</iyy><izz>5e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="col"><geometry><box><size>0.02 0.03 0.2</size></box></geometry></collision>
        <visual name="vis">
          <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
          <geometry><box><size>0.02 0.03 0.2</size></box></geometry>
        </visual>
      </link>
    </model>
"""
    )

    parts.append("  </world>\n</sdf>\n")

    pkg_root = Path(__file__).resolve().parent.parent
    world_path = pkg_root / "worlds" / "maze_world.world"
    world_path.write_text("".join(parts), encoding="utf-8")
    print(f"Wrote {world_path} ({idx} wall segments, objects at cell ({row},{col}))")


if __name__ == "__main__":
    main()
