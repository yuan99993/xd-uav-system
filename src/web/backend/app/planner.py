"""A* 地形规划器（最简版本，先打通链路用）。

为什么先不用 RRT*/Dubins：
    terrain-navigation 原项目用 RRT* in Dubins Airplane 空间，
    那个实现复杂，零基础一周写不出来。
    我们第一版用 A* 在水平网格上规划，不考虑 Dubins 曲率约束。
    优先级是"链路跑通"，后面再升级算法。

对应原项目代码：
    terrain_planner/src/terrain_ompl_rrt.cpp
    （这里我们做了简化，等链路稳定再换 RRT*）
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .terrain import TerrainMap


@dataclass(order=True)
class _Node:
    """A* 优先队列节点"""
    f: float                          # f = g + h，估计总代价
    g: float = field(compare=False)   # 已走代价
    pos: Tuple[int, int] = field(compare=False)  # (row, col)
    parent: Optional[Tuple[int, int]] = field(compare=False, default=None)


@dataclass
class PathSegment:
    """对应 ROS 消息 planner_msgs/PathSegment 的 Python 表示

    字段对齐 terrain-navigation/planner_msgs/msg/PathSegment.msg：
        segment_start:    geometry_msgs/Vector3   起点xyz
        segment_tangent:  geometry_msgs/Vector3   切向xyz
        curvature:        Float64                   曲率（直线=0）
        segment_length:   Float64                   段长（米）
        is_periodic:      bool                      是不是整圆
        reached:          bool                      到达标志
    """
    segment_start: Tuple[float, float, float]
    segment_tangent: Tuple[float, float, float]
    curvature: float
    segment_length: float
    is_periodic: bool = False
    reached: bool = False


@dataclass
class PathResult:
    """对应 ROS 消息 planner_msgs/Path 的 Python 表示"""
    segments: List[PathSegment]
    waypoints: List[Tuple[float, float, float]]  # 调试用：xyz 折线点列
    length: float


class TerrainAStarPlanner:
    """A* 在水平网格上做地形安全规划。

    思路：
        1. 把地形网格当成图，每个格子是节点
        2. 邻居 = 8 邻域（上下左右 + 4 对角）
        3. 代价 = 水平距离 + 高度惩罚（爬升贵、下降便宜）
        4. 启发式 = 到终点的水平欧氏距离
        5. 撞山的格子直接禁用（is_collision == True）
        6. 禁飞区内的格子直接禁用（点-多边形测试）
    """

    def __init__(self, terrain: TerrainMap, cruise_altitude: float = 100.0,
                 safety_margin: float = 30.0):
        """
        Args:
            terrain: 已加载的地形
            cruise_altitude: 巡航高度（高于海平面，米）
            safety_margin: 安全余量（米），飞机必须高于山顶这么高
        """
        self.terrain = terrain
        self.cruise_altitude = cruise_altitude
        self.safety_margin = safety_margin

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        """射线法判断点 (x, y) 是否在多边形内部。

        经典点-多边形测试：从该点向右发一条射线，
        与多边形边的交点数为奇数 → 在内部；偶数 → 在外部。
        planner3d 的 RRT* 也复用本函数做禁飞区检查。
        """
        n = len(polygon)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            # 判断射线是否穿过这条边
            if ((yi > y) != (yj > y)) and \
               (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
            j = i
        return inside

    def plan(self, start_xy: Tuple[float, float],
             goal_xy: Tuple[float, float],
             nofly_zones: Optional[List[List[Tuple[float, float]]]] = None
             ) -> Optional[PathResult]:
        """规划一条从 start 到 goal 的路径。

        Args:
            start_xy, goal_xy: 地理坐标 (x=东/经度, y=北/纬度)
            nofly_zones: 禁飞区列表，每个禁飞区是多边形顶点列表 [(x,y), ...]

        Returns:
            PathResult 或 None（找不到路径）
        """
        if nofly_zones is None:
            nofly_zones = []

        gt = self.terrain.geo_transform
        # 地理坐标 → 网格 (row, col)
        def to_rc(x, y):
            col = int((x - gt[0]) / gt[1])
            row = int((y - gt[3]) / gt[5])
            return row, col
        def to_xy(row, col):
            x = gt[0] + col * gt[1]
            y = gt[3] + row * gt[5]
            return x, y

        start_rc = to_rc(*start_xy)
        goal_rc = to_rc(*goal_xy)
        if not self._in_grid(start_rc) or not self._in_grid(goal_rc):
            raise ValueError("起点或终点超出地图范围")

        # 8 邻域偏移
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]

        open_heap = [_Node(f=0.0, g=0.0, pos=start_rc, parent=None)]
        came_from = {start_rc: None}
        g_score = {start_rc: 0.0}

        rows, cols = self.terrain.rows, self.terrain.cols
        elev = self.terrain.elevation

        # ---- 距离全部换算成"米"，保证代价/启发式单位一致 ----
        # 否则水平步长是"度"(~1e-5)、爬升是"米"(几十)，A* 会被高度项主导、
        # 同一格反复入堆弹出，轻易耗尽迭代上限而误判"找不到路径"。
        lat_center = gt[3] + gt[5] * rows / 2.0
        m_per_deg_lat = 111320.0
        m_per_deg_lng = 111320.0 * math.cos(math.radians(lat_center))
        cell_x_m = abs(gt[1]) * m_per_deg_lng  # 一格东西向米数
        cell_y_m = abs(gt[5]) * m_per_deg_lat  # 一格南北向米数
        if cell_x_m == 0:
            cell_x_m = 1.0
        if cell_y_m == 0:
            cell_y_m = 1.0

        iters = 0
        max_iters = 4 * rows * cols  # 放宽上限（同一格可能多次入堆）
        while open_heap and iters < max_iters:
            iters += 1
            cur = heapq.heappop(open_heap)
            if cur.pos == goal_rc:
                return self._reconstruct(cur.pos, came_from, to_xy,
                                         m_per_deg_lng, m_per_deg_lat)

            # 如果这个节点已经被更优路径访问过，跳过
            if cur.g > g_score.get(cur.pos, math.inf):
                continue

            for dr, dc in nbrs:
                nr, nc = cur.pos[0] + dr, cur.pos[1] + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                # 撞山检查（核心：对应 isInCollision）
                elev_here = elev[nr, nc]
                if np.isnan(elev_here):
                    continue
                if elev_here + self.safety_margin > self.cruise_altitude:
                    continue  # 这格山顶超过飞机 → 禁用

                # 禁飞区检查：该格中心点是否落在任何禁飞区内
                cell_x, cell_y = to_xy(nr, nc)
                in_nofly = False
                for zone in nofly_zones:
                    if self._point_in_polygon(cell_x, cell_y, zone):
                        in_nofly = True
                        break
                if in_nofly:
                    continue  # 这格在禁飞区内 → 禁用

                # 水平步长（米）：正交/对角统一用勾股
                step = math.hypot(dc * cell_x_m, dr * cell_y_m)
                # 爬升代价（米）：只惩罚往上爬，下降不花钱
                d_alt = float(elev_here - elev[cur.pos])
                cost = step + max(0.0, d_alt) * 0.5
                tentative = cur.g + cost

                key = (nr, nc)
                if tentative < g_score.get(key, math.inf):
                    g_score[key] = tentative
                    came_from[key] = cur.pos
                    # 启发式：到终点的水平直线距离（米），admissible 且一致
                    h = math.hypot((goal_rc[1] - nc) * cell_x_m,
                                   (goal_rc[0] - nr) * cell_y_m)
                    heapq.heappush(open_heap, _Node(
                        f=tentative + h, g=tentative, pos=key, parent=cur.pos))
        return None  # 找不到路径

    def _in_grid(self, rc):
        return 0 <= rc[0] < self.terrain.rows and 0 <= rc[1] < self.terrain.cols

    def _reconstruct(self, goal_rc, came_from, to_xy,
                     m_per_deg_lng, m_per_deg_lat) -> PathResult:
        # 回溯成路径点列
        path_rc = []
        cur = goal_rc
        while cur is not None:
            path_rc.append(cur)
            cur = came_from.get(cur)
        path_rc.reverse()

        # 转成 (x, y, alt)
        waypoints = [(to_xy(r, c)[0], to_xy(r, c)[1], self.cruise_altitude)
                     for r, c in path_rc]

        # 聚合成 PathSegment（每两个相邻点一段直线，曲率=0）
        segments = []
        total_len = 0.0
        for i in range(len(waypoints) - 1):
            p0 = waypoints[i]
            p1 = waypoints[i + 1]
            # 经纬度差 → 米，保证段长/切向单位正确
            mx = (p1[0] - p0[0]) * m_per_deg_lng
            my = (p1[1] - p0[1]) * m_per_deg_lat
            seg_len = math.hypot(mx, my)
            if seg_len < 1e-9:
                continue
            tx, ty = mx / seg_len, my / seg_len
            segments.append(PathSegment(
                segment_start=p0,
                segment_tangent=(tx, ty, 0.0),
                curvature=0.0,
                segment_length=seg_len,
            ))
            total_len += seg_len
        return PathResult(segments=segments, waypoints=waypoints, length=total_len)
