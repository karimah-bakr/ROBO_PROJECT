"""LIDAR processor node.

Consumes /scan (sensor_msgs/LaserScan) and produces two outputs:

  /wall_status      std_msgs/String JSON   {"F":bool, "R":bool, "B":bool, "L":bool,
                                            "ranges":{"F":float, ...}}
  /object_detected  std_msgs/String JSON   {"angle_deg":..., "distance_m":...,
                                            "width_m":...}

Sectors (degrees, robot-relative, 0 = forward, CCW positive):
    FRONT  : -30 ..  30
    RIGHT  :  60 .. 120
    BACK   : 150 .. 210
    LEFT   : 240 .. 300

The node is purely reactive: every scan triggers one /wall_status publish.
Object detection looks for short, narrow clusters (matching the 2cm cube
profile) that sit away from the wall_threshold band.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass
class Sector:
    name: str
    min_deg: float
    max_deg: float


class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_processor")

        # --- parameters ---
        defaults: Dict[str, float] = {
            "front_min_deg": -30.0, "front_max_deg": 30.0,
            "right_min_deg":  60.0, "right_max_deg": 120.0,
            "back_min_deg":  150.0, "back_max_deg":  210.0,
            "left_min_deg":  240.0, "left_max_deg":  300.0,
            "wall_threshold_m": 0.25,
            "min_valid_range_m": 0.06,
            "max_valid_range_m": 3.50,
            "object_cluster_min_points": 2,
            "object_cluster_max_points": 10,
            "object_cluster_max_width_m": 0.06,
            "object_min_distance_m": 0.08,
            "object_max_distance_m": 0.45,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)

        def p(name: str) -> float:
            return float(self.get_parameter(name).value)

        self.sectors = [
            Sector("F", p("front_min_deg"), p("front_max_deg")),
            Sector("R", p("right_min_deg"), p("right_max_deg")),
            Sector("B", p("back_min_deg"),  p("back_max_deg")),
            Sector("L", p("left_min_deg"),  p("left_max_deg")),
        ]
        self.wall_threshold = p("wall_threshold_m")
        self.min_valid = p("min_valid_range_m")
        self.max_valid = p("max_valid_range_m")
        self.obj_min_pts = int(p("object_cluster_min_points"))
        self.obj_max_pts = int(p("object_cluster_max_points"))
        self.obj_max_width = p("object_cluster_max_width_m")
        self.obj_min_dist = p("object_min_distance_m")
        self.obj_max_dist = p("object_max_distance_m")

        # --- I/O ---
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.wall_pub = self.create_publisher(String, "/wall_status", 10)
        self.obj_pub = self.create_publisher(String, "/object_detected", 10)

        self._last_scan_stamp = self.get_clock().now()
        self.create_timer(2.0, self._check_dropout)
        self.get_logger().info("lidar_processor ready")

    # ------------------------------------------------------------------

    def _check_dropout(self) -> None:
        now = self.get_clock().now()
        gap = (now - self._last_scan_stamp).nanoseconds * 1e-9
        if gap > 2.0:
            self.get_logger().warn(f"no LaserScan for {gap:.1f}s")

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_stamp = self.get_clock().now()
        try:
            sector_ranges = self._collect_sectors(msg)
            wall_msg = {"ranges": {}, "F": False, "R": False, "B": False, "L": False}
            for sec in self.sectors:
                pts = sector_ranges[sec.name]
                if not pts:
                    wall_msg["ranges"][sec.name] = float("inf")
                    continue
                # use the closest valid range in the sector
                rmin = min(r for _, r in pts)
                wall_msg["ranges"][sec.name] = rmin
                wall_msg[sec.name] = rmin < self.wall_threshold
            self.wall_pub.publish(String(data=json.dumps(wall_msg)))

            # object detection — only meaningful in the FRONT sector here
            obj = self._detect_object(sector_ranges["F"])
            if obj is not None:
                self.obj_pub.publish(String(data=json.dumps(obj)))
        except Exception as exc:  # graceful: never crash on a bad scan
            self.get_logger().warn(f"scan processing failed: {exc}")

    # ------------------------------------------------------------------

    def _collect_sectors(self, msg: LaserScan) -> Dict[str, List[Tuple[float, float]]]:
        """Return {sector_name: [(angle_deg, range_m), ...]} for valid ranges."""
        out: Dict[str, List[Tuple[float, float]]] = {s.name: [] for s in self.sectors}
        if not msg.ranges:
            return out
        n = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_valid or r > self.max_valid:
                continue
            ang_rad = msg.angle_min + i * msg.angle_increment
            ang_deg = math.degrees(ang_rad) % 360.0
            for sec in self.sectors:
                if _angle_in_sector(ang_deg, sec.min_deg, sec.max_deg):
                    out[sec.name].append((ang_deg, r))
                    break
            _ = n  # keep n referenced for future debug
        return out

    def _detect_object(
        self, front_points: List[Tuple[float, float]]
    ) -> Optional[Dict[str, float]]:
        """Find a narrow cluster in the front sector matching the cube profile."""
        if len(front_points) < self.obj_min_pts:
            return None
        # sort by angle so adjacent points sit next to each other
        pts = sorted(front_points, key=lambda t: t[0])
        clusters: List[List[Tuple[float, float]]] = [[pts[0]]]
        for ang, r in pts[1:]:
            prev_ang, prev_r = clusters[-1][-1]
            if abs(ang - prev_ang) < 3.0 and abs(r - prev_r) < 0.05:
                clusters[-1].append((ang, r))
            else:
                clusters.append([(ang, r)])

        for cluster in clusters:
            if not (self.obj_min_pts <= len(cluster) <= self.obj_max_pts):
                continue
            mean_r = sum(r for _, r in cluster) / len(cluster)
            if not (self.obj_min_dist <= mean_r <= self.obj_max_dist):
                continue
            # angular width → arc width at mean distance
            ang_span = abs(cluster[-1][0] - cluster[0][0])
            arc_w = math.radians(ang_span) * mean_r
            if arc_w > self.obj_max_width:
                continue
            mean_ang = sum(a for a, _ in cluster) / len(cluster)
            # wrap to (-180, 180]
            if mean_ang > 180.0:
                mean_ang -= 360.0
            return {
                "angle_deg": mean_ang,
                "distance_m": mean_r,
                "width_m": arc_w,
                "n_points": len(cluster),
            }
        return None


def _angle_in_sector(ang_deg: float, lo_deg: float, hi_deg: float) -> bool:
    """Handle the FRONT case where lo < 0 (wraps around 360)."""
    if lo_deg < 0:
        return ang_deg >= (360.0 + lo_deg) or ang_deg <= hi_deg
    return lo_deg <= ang_deg <= hi_deg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
