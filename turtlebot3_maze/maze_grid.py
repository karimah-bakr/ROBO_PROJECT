#!/usr/bin/env python3
"""
maze_grid.py
============
خريطة المتاهة 7×7 مع:
  - تسجيل الجدران من LIDAR
  - خوارزمية A* لإيجاد أقصر مسار
  - تتبع الخلايا المزارة
"""

import heapq

# ───────────────────────────── ثوابت ─────────────────────────────
CELL  = 0.7   # حجم الخلية بالمتر
N     = 7     # عدد الصفوف والأعمدة

WALL_N  = 1
WALL_S  = 2
WALL_E  = 4
WALL_W  = 8
VISITED = 16

# الاتجاهات: التغيير في (row, col)
DR       = {"N": 1,  "S": -1, "E": 0,  "W": 0}
DC       = {"N": 0,  "S": 0,  "E": 1,  "W": -1}
OPPOSITE = {"N": "S","S": "N","E": "W","W": "E"}
DIR_BIT  = {"N": WALL_N, "S": WALL_S, "E": WALL_E, "W": WALL_W}


class MazeGrid:
    """تمثيل الشبكة + خوارزمية A*."""

    def __init__(self):
        self.grid = [[0] * N for _ in range(N)]

    # ─────────────── تحويل إحداثيات ───────────────
    def rc(self, x: float, y: float) -> tuple[int, int]:
        """تحويل (x, y) بالمتر إلى (row, col)."""
        col = max(0, min(N - 1, int(x / CELL)))
        row = max(0, min(N - 1, int(y / CELL)))
        return row, col

    def cell_center(self, r: int, c: int) -> tuple[float, float]:
        """مركز الخلية بالمتر (x, y)."""
        return c * CELL + CELL / 2, r * CELL + CELL / 2

    # ─────────────── تعديل الشبكة ───────────────
    def set_wall(self, r: int, c: int, d: str):
        """تسجيل جدار في الاتجاه d من الخلية (r, c)."""
        if 0 <= r < N and 0 <= c < N:
            self.grid[r][c] |= DIR_BIT[d]
            nr, nc = r + DR[d], c + DC[d]
            if 0 <= nr < N and 0 <= nc < N:
                self.grid[nr][nc] |= DIR_BIT[OPPOSITE[d]]

    def mark_visited(self, r: int, c: int):
        if 0 <= r < N and 0 <= c < N:
            self.grid[r][c] |= VISITED

    # ─────────────── استعلامات ───────────────
    def has_wall(self, r: int, c: int, d: str) -> bool:
        if not (0 <= r < N and 0 <= c < N):
            return True
        return bool(self.grid[r][c] & DIR_BIT[d])

    def is_visited(self, r: int, c: int) -> bool:
        return bool(self.grid[r][c] & VISITED)

    def neighbours(self, r: int, c: int) -> list[tuple[int, int]]:
        """الخلايا المجاورة بدون جدار."""
        out = []
        for d in ("N", "S", "E", "W"):
            if not self.has_wall(r, c, d):
                nr, nc = r + DR[d], c + DC[d]
                if 0 <= nr < N and 0 <= nc < N:
                    out.append((nr, nc))
        return out

    # ─────────────── A* ───────────────
    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """
        إرجاع أقصر مسار من start إلى goal كقائمة خلايا.
        إذا لم يوجد مسار → قائمة فارغة.
        """
        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_heap = [(h(start, goal), 0, start)]
        came_from: dict = {}
        g_score = {start: 0}

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)
            if cur == goal:
                path = []
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                return list(reversed(path))

            if g > g_score.get(cur, float('inf')):
                continue

            for nb in self.neighbours(*cur):
                ng = g + 1
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (ng + h(nb, goal), ng, nb))

        return []  # لا يوجد مسار

    def unvisited_neighbours(self, r: int, c: int) -> list[tuple[int, int]]:
        """الخلايا المجاورة غير المزارة (للاستكشاف)."""
        return [(nr, nc) for nr, nc in self.neighbours(r, c)
                if not self.is_visited(nr, nc)]

    def print_grid(self, logger=None):
        """طباعة الشبكة للتشخيص."""
        lines = []
        for r in range(N - 1, -1, -1):
            row_str = ""
            for c in range(N):
                v = "V" if self.is_visited(r, c) else "."
                row_str += f"[{v}]"
            lines.append(row_str)
        msg = "\n" + "\n".join(lines)
        if logger:
            logger.info(msg)
        else:
            print(msg)