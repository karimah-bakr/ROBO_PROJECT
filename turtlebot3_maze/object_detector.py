#!/usr/bin/env python3
"""
object_detector.py
==================
كشف الأجسام باستخدام LIDAR:
  - تمييز الأجسام الصغيرة (2×3×20 cm) عن الجدران
  - تحديد موضع الجسم بالنسبة للروبوت
  - تسجيل مواضع الأجسام المكتشفة على الخريطة
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ───────────── ثوابت الكشف ─────────────
WALL_DIST       = 0.28   # أقل من هذا → جدار
OBJ_MAX_DIST    = 0.45   # أقصى مسافة للكشف عن الجسم
OBJ_MIN_DIST    = 0.05   # أدنى مسافة (لتفادي الضجيج)
OBJ_MAX_WIDTH   = 0.08   # أقصى عرض للجسم (بالراديان × المسافة)
OBJ_MIN_WIDTH   = 0.01   # أدنى عرض للجسم


class ObjectDetector:
    """كشف الأجسام من بيانات LIDAR."""

    def __init__(self, node: Node):
        self.node = node
        self.found_objects: list[tuple[float, float]] = []   # [(x,y), ...]
        self._last_scan = None

        node.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _scan_cb(self, msg: LaserScan):
        self._last_scan = msg

    # ─────────────── الكشف ───────────────
    def detect_object_ahead(self) -> tuple[float, float] | None:
        """
        يفحص الـ scan الأخير ويعيد (angle, distance) للجسم أمام الروبوت.
        يعيد None إذا لم يجد جسماً.
        """
        if self._last_scan is None:
            return None

        msg = self._last_scan
        r = msg.ranges
        n = len(r)

        # نبحث في القوس الأمامي ±30°
        clusters = []
        cluster  = []

        for i in range(-30, 31):
            idx = i % n
            v   = r[idx]
            if math.isfinite(v) and OBJ_MIN_DIST < v < OBJ_MAX_DIST:
                cluster.append((i, v))
            else:
                if cluster:
                    clusters.append(cluster)
                    cluster = []
        if cluster:
            clusters.append(cluster)

        best = None
        for cl in clusters:
            if len(cl) < 2:
                continue
            angles = [a for a, _ in cl]
            dists  = [d for _, d in cl]

            # حساب العرض الزاوي
            angle_span = (max(angles) - min(angles)) * math.pi / 180.0
            avg_dist   = sum(dists) / len(dists)
            width      = angle_span * avg_dist   # عرض تقريبي بالمتر

            if OBJ_MIN_WIDTH < width < OBJ_MAX_WIDTH:
                mid_angle = sum(angles) / len(angles) * math.pi / 180.0
                if best is None or avg_dist < best[1]:
                    best = (mid_angle, avg_dist)

        return best

    def object_in_range(self, threshold: float = 0.30) -> bool:
        """هل يوجد جسم في مدى قريب؟"""
        result = self.detect_object_ahead()
        return result is not None and result[1] < threshold

    # ─────────────── تسجيل المواضع ───────────────
    def register_object(self, robot_x: float, robot_y: float, robot_yaw: float):
        """
        تحويل موضع الجسم المكتشف من إطار الروبوت إلى الخريطة.
        """
        result = self.detect_object_ahead()
        if result is None:
            return None

        angle, dist = result
        global_angle = robot_yaw + angle
        ox = robot_x + dist * math.cos(global_angle)
        oy = robot_y + dist * math.sin(global_angle)

        # تحقق إن الموضع مش مكرر
        for fx, fy in self.found_objects:
            if math.hypot(ox - fx, oy - fy) < 0.25:
                return (fx, fy)   # نفس الجسم

        self.found_objects.append((ox, oy))
        self.node.get_logger().info(
            f'🎯 تم اكتشاف جسم في ({ox:.2f}, {oy:.2f})'
        )
        return (ox, oy)

    def get_found_count(self) -> int:
        return len(self.found_objects)