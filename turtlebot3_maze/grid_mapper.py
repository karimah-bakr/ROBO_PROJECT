"""Grid mapper node.

Maintains a 7x7 occupancy / wall map of the maze. Other nodes feed it
*cell observations* (the robot's current cell + which of the four
cardinal directions show a wall). It applies symmetric updates (if a
wall exists on the north side of cell A, it also exists on the south
side of cell B = A's northern neighbour) and republishes the full map
as JSON on /maze_map.

Cell encoding (single uint8, bitwise OR of these flags):
    WALL_N  = 1
    WALL_S  = 2
    WALL_E  = 4
    WALL_W  = 8
    VISITED = 16

Coordinate frame: row 1 = south, col 1 = west.
"""

from __future__ import annotations

import json
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


WALL_N = 1
WALL_S = 2
WALL_E = 4
WALL_W = 8
VISITED = 16

DIR_BIT = {"N": WALL_N, "S": WALL_S, "E": WALL_E, "W": WALL_W}
OPPOSITE = {"N": "S", "S": "N", "E": "W", "W": "E"}
DR = {"N": 1, "S": -1, "E": 0, "W": 0}
DC = {"N": 0, "S": 0, "E": 1, "W": -1}


class MazeGrid:
    """Pure-Python maze grid; no ROS dependency so other nodes can import it."""

    def __init__(self, rows: int = 7, cols: int = 7):
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=np.uint8)
        self.path_history: List[Tuple[int, int]] = []
        self._mark_outer_boundary()

    def _mark_outer_boundary(self) -> None:
        for c in range(self.cols):
            self.grid[0, c] |= WALL_S
            self.grid[self.rows - 1, c] |= WALL_N
        for r in range(self.rows):
            self.grid[r, 0] |= WALL_W
            self.grid[r, self.cols - 1] |= WALL_E

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def set_wall(self, row: int, col: int, direction: str, present: bool) -> None:
        """Add (or clear) a wall, mirrored on the neighbour."""
        if not self.in_bounds(row, col):
            return
        bit = DIR_BIT[direction]
        if present:
            self.grid[row, col] |= bit
        # We intentionally don't clear a previously-known wall on a single
        # "no-wall" observation — a noisy LIDAR frame can fail to see a
        # real wall, but a real wall doesn't disappear.

        nr, nc = row + DR[direction], col + DC[direction]
        if self.in_bounds(nr, nc):
            obit = DIR_BIT[OPPOSITE[direction]]
            if present:
                self.grid[nr, nc] |= obit

    def mark_visited(self, row: int, col: int) -> None:
        if not self.in_bounds(row, col):
            return
        self.grid[row, col] |= VISITED
        if not self.path_history or self.path_history[-1] != (row, col):
            self.path_history.append((row, col))

    def is_visited(self, row: int, col: int) -> bool:
        return bool(self.grid[row, col] & VISITED) if self.in_bounds(row, col) else False

    def has_wall(self, row: int, col: int, direction: str) -> bool:
        if not self.in_bounds(row, col):
            return True
        return bool(self.grid[row, col] & DIR_BIT[direction])

    def neighbours(self, row: int, col: int) -> List[Tuple[Tuple[int, int], str]]:
        """Return reachable (nr, nc, direction) neighbours (no wall between)."""
        out: List[Tuple[Tuple[int, int], str]] = []
        for d in ("N", "S", "E", "W"):
            if self.has_wall(row, col, d):
                continue
            nr, nc = row + DR[d], col + DC[d]
            if self.in_bounds(nr, nc):
                out.append(((nr, nc), d))
        return out

    def to_dict(self) -> Dict:
        return {
            "rows": self.rows,
            "cols": self.cols,
            "grid": self.grid.tolist(),
            "path": self.path_history,
        }

    @classmethod
    def from_dict(cls, payload: Dict) -> "MazeGrid":
        g = cls(int(payload["rows"]), int(payload["cols"]))
        g.grid = np.array(payload["grid"], dtype=np.uint8)
        g.path_history = [tuple(p) for p in payload.get("path", [])]
        return g


class GridMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("grid_mapper")
        self.declare_parameter("rows", 7)
        self.declare_parameter("cols", 7)
        self.declare_parameter("publish_period_s", 0.5)

        rows = int(self.get_parameter("rows").value)
        cols = int(self.get_parameter("cols").value)
        period = float(self.get_parameter("publish_period_s").value)

        self.grid = MazeGrid(rows, cols)

        self.map_pub = self.create_publisher(String, "/maze_map", 10)
        self.obs_sub = self.create_subscription(
            String, "/cell_observation", self._on_observation, 20
        )
        self.timer = self.create_timer(period, self._publish_map)
        self.get_logger().info(f"grid_mapper ready ({rows}x{cols})")

    def _on_observation(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f"bad observation JSON: {exc}")
            return

        row = int(data.get("row", -1))
        col = int(data.get("col", -1))
        if not self.grid.in_bounds(row, col):
            self.get_logger().warn(f"observation out of bounds: ({row},{col})")
            return

        self.grid.mark_visited(row, col)
        walls = data.get("walls") or {}
        for d in ("N", "S", "E", "W"):
            present = bool(walls.get(d, False))
            if present:
                self.grid.set_wall(row, col, d, True)
                self.get_logger().info(
                    f"Wall detected: {'NORTH' if d=='N' else 'SOUTH' if d=='S' else 'EAST' if d=='E' else 'WEST'} at ({row},{col})"
                )

    def _publish_map(self) -> None:
        try:
            payload = json.dumps(self.grid.to_dict())
        except Exception as exc:  # pragma: no cover  (defensive)
            self.get_logger().error(f"map serialize failed: {exc}")
            return
        self.map_pub.publish(String(data=payload))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
