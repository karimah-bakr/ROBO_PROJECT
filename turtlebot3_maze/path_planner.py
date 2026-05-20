"""Path planner node + library.

Library:
    astar(grid, start, goal)                -> list[(r,c)] | None
    bfs_nearest_unvisited(grid, start)      -> list[(r,c)] | None
    cells_to_moves(cells, start_heading)    -> list[str], end_heading

Node:
    Subscribes:  /path_request  (std_msgs/String JSON)
    Subscribes:  /maze_map      (std_msgs/String JSON)  -- latest grid snapshot
    Publishes:   /path_result   (std_msgs/String JSON)

Request JSON:
    {"id": "<unique>",
     "mode": "astar" | "explore",
     "start": [row, col],
     "goal":  [row, col] | null,
     "heading": "N" | "S" | "E" | "W"}

Result JSON:
    {"id": "<same>",
     "ok": true | false,
     "moves": ["FORWARD", "TURN_LEFT", ...],
     "cells": [[r,c], ...],
     "end_heading": "N",
     "reason": "<optional>"}
"""

from __future__ import annotations

import heapq
import json
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtlebot3_maze.grid_mapper import (
    MazeGrid, DR, DC, OPPOSITE, VISITED,
)


HEADINGS = ("N", "E", "S", "W")
# index: 0=N, 1=E, 2=S, 3=W -- 90° CW step per increment.


def _delta_to_heading(dr: int, dc: int) -> Optional[str]:
    if dr == 1 and dc == 0:
        return "N"
    if dr == -1 and dc == 0:
        return "S"
    if dr == 0 and dc == 1:
        return "E"
    if dr == 0 and dc == -1:
        return "W"
    return None


def _turn_command(curr: str, target: str) -> Optional[str]:
    """Return a single turn command (or None if already aligned)."""
    if curr == target:
        return None
    i = HEADINGS.index(curr)
    j = HEADINGS.index(target)
    diff = (j - i) % 4
    return {1: "TURN_RIGHT", 2: "TURN_180", 3: "TURN_LEFT"}[diff]


# ---------------------------------------------------------------------------
# A* on the known map
# ---------------------------------------------------------------------------

def astar(grid: MazeGrid,
          start: Tuple[int, int],
          goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """Manhattan-heuristic A* through the known wall map.

    Returns the cell list (including start, ending at goal) or None.
    """
    if start == goal:
        return [start]
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None

    def h(c: Tuple[int, int]) -> int:
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    open_heap: List[Tuple[int, int, Tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (h(start), counter, start))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], int] = {start: 0}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(came_from, current)
        for (nbr, _direction) in grid.neighbours(*current):
            tentative = g_score[current] + 1
            if tentative < g_score.get(nbr, 10**9):
                came_from[nbr] = current
                g_score[nbr] = tentative
                counter += 1
                f = tentative + h(nbr)
                heapq.heappush(open_heap, (f, counter, nbr))
    return None


def _reconstruct(came_from: Dict[Tuple[int, int], Tuple[int, int]],
                 end: Tuple[int, int]) -> List[Tuple[int, int]]:
    out = [end]
    while end in came_from:
        end = came_from[end]
        out.append(end)
    out.reverse()
    return out


# ---------------------------------------------------------------------------
# BFS exploration: nearest unvisited reachable cell
# ---------------------------------------------------------------------------

def bfs_nearest_unvisited(grid: MazeGrid,
                          start: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """Return a BFS path from start to the closest cell whose VISITED bit
    is not yet set (and which is reachable through the discovered walls).

    For exploration, we treat *unknown* directions (no wall observed yet,
    cell on the other side not visited) as passable — that's the point.
    """
    if not grid.in_bounds(*start):
        return None
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    seen = {start}
    q: deque = deque([start])
    while q:
        current = q.popleft()
        if current != start and not grid.is_visited(*current):
            return _reconstruct(came_from, current)
        r, c = current
        for d in ("N", "E", "S", "W"):
            if grid.has_wall(r, c, d):
                continue
            nr, nc = r + DR[d], c + DC[d]
            if not grid.in_bounds(nr, nc):
                continue
            if (nr, nc) in seen:
                continue
            seen.add((nr, nc))
            came_from[(nr, nc)] = current
            q.append((nr, nc))
    return None


# ---------------------------------------------------------------------------
# Cell-list → move-list
# ---------------------------------------------------------------------------

def cells_to_moves(cells: List[Tuple[int, int]],
                   start_heading: str) -> Tuple[List[str], str]:
    """Convert a contiguous cell list (length >= 1) into a flat move list.

    Each step emits zero or one turn followed by a FORWARD.
    Returns (moves, final_heading).
    """
    moves: List[str] = []
    heading = start_heading
    if len(cells) < 2:
        return moves, heading

    for (r0, c0), (r1, c1) in zip(cells[:-1], cells[1:]):
        target = _delta_to_heading(r1 - r0, c1 - c0)
        if target is None:
            raise ValueError(f"non-adjacent cells in path: {(r0,c0)} -> {(r1,c1)}")
        turn = _turn_command(heading, target)
        if turn:
            moves.append(turn)
            heading = target
        moves.append("FORWARD")
    return moves, heading


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_planner")
        self.declare_parameter("rows", 7)
        self.declare_parameter("cols", 7)
        rows = int(self.get_parameter("rows").value)
        cols = int(self.get_parameter("cols").value)
        self.grid = MazeGrid(rows, cols)

        self.req_sub = self.create_subscription(String, "/path_request", self._on_request, 10)
        self.map_sub = self.create_subscription(String, "/maze_map", self._on_map, 10)
        self.res_pub = self.create_publisher(String, "/path_result", 10)
        self.get_logger().info("path_planner ready")

    def _on_map(self, msg: String) -> None:
        try:
            self.grid = MazeGrid.from_dict(json.loads(msg.data))
        except Exception as exc:
            self.get_logger().warn(f"bad /maze_map: {exc}")

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"bad /path_request: {exc}")
            return

        rid = req.get("id", "")
        mode = req.get("mode", "astar")
        start = tuple(req.get("start", [0, 0]))
        heading = req.get("heading", "N")

        try:
            if mode == "astar":
                goal = tuple(req["goal"])
                cells = astar(self.grid, start, goal)
            elif mode == "explore":
                cells = bfs_nearest_unvisited(self.grid, start)
            else:
                cells = None
        except Exception as exc:
            self.get_logger().warn(f"planning crashed: {exc}")
            cells = None

        if cells is None:
            self._send(rid, ok=False, reason="no path found")
            return
        try:
            moves, end_heading = cells_to_moves(cells, heading)
        except Exception as exc:
            self._send(rid, ok=False, reason=f"move conversion failed: {exc}")
            return
        self._send(rid, ok=True, moves=moves, cells=cells, end_heading=end_heading)

    def _send(self, rid: str, ok: bool, moves: Optional[List[str]] = None,
              cells: Optional[List[Tuple[int, int]]] = None,
              end_heading: str = "N", reason: str = "") -> None:
        payload = {
            "id": rid,
            "ok": ok,
            "moves": moves or [],
            "cells": [list(c) for c in (cells or [])],
            "end_heading": end_heading,
            "reason": reason,
        }
        self.res_pub.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# Keep VISITED / OPPOSITE referenced for re-export clarity to callers.
_ = (VISITED, OPPOSITE)
