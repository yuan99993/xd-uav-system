"""RRT* + Dubins Airplane 全局规划器（对齐 terrain-navigation TerrainOmplRrt）。

对应原项目代码：
    terrain_planner/src/terrain_ompl_rrt.cpp      # RRT* 求解、solutionPathToPath
    terrain_planner/src/terrain_ompl.cpp          # TerrainValidityChecker / TerrainStateSampler
    terrain_planner/src/terrain_planner.cpp       # 高度重排（贴地飞行）

核心逻辑（与原项目一一对应）：
    1. 状态空间 = Dubins Airplane (x, y, z, yaw)，见 dubins_airplane.py
    2. 采样器：z = 地形高程 + uniform(AGL_min, AGL_max) —— 相对地形采样
    3. 有效性：沿 Dubins 路径离散采样做 isInCollision（地面层 + 天花板层）
    4. 目标：终点悬停圆（Dubins-to-circle，正/负半径 = 左/右盘旋）
    5. RRT*：choose-parent + rewire，时间预算内持续优化
    6. 求解后高度重排：整条路径 z 贴地形 + 爬升率限制（地形跟随）
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .dubins_airplane import DubinsAirplanePath, dubins_airplane, state_at, walk_path
from .planner import TerrainAStarPlanner
from .terrain import TerrainMap

State = Tuple[float, float, float, float]  # (x, y, z, yaw) ENU 米 + 海拔米 + 弧度


@dataclass
class Plan3DParams:
    """3D 规划参数（全部米制 / 弧度入口用度）。"""
    agl_min: float = 50.0          # 离地高度下限（米）
    agl_max: float = 150.0         # 离地高度上限（天花板）
    turning_radius: float = 66.67  # 最小转弯半径（米）
    climb_angle_deg: float = 10.0  # 最大爬升/俯冲角（度）
    safety_margin: float = 30.0    # 地面安全余量（米）
    time_budget: float = 3.0       # 规划时间预算（秒）
    step: float = 250.0            # RRT* 单步最大扩展长度（米）
    goal_bias: float = 0.10        # 朝目标集采样的概率
    loiter: str = 'ccw'            # 终点盘旋方向：'ccw' | 'cw' | 'none'
    nofly_zones: List[List[Tuple[float, float]]] = field(default_factory=list)


@dataclass
class Plan3DResult:
    waypoints: List[Tuple[float, float, float]]  # (lng, lat, z) 稀疏输出
    segments: List[dict]                         # 直线段（对齐 PathSegment 字段）
    length: float                                # 3D 总长（米）
    iterations: int
    plan_time: float
    solution_cost: float                         # RRT* 的 Dubins 代价


class _Node:
    __slots__ = ("state", "parent", "cost", "children")

    def __init__(self, state: State, parent: Optional["_Node"] = None, cost: float = 0.0):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.children: List[_Node] = []


class TerrainRRTStar3D:
    """地形 RRT* 规划器（ENU 米制空间）。"""

    def __init__(self, terrain: TerrainMap, params: Plan3DParams):
        self.terrain = terrain
        self.p = params
        self.gamma = math.radians(params.climb_angle_deg)
        self.rho = params.turning_radius
        self.bounds = terrain.enu_bounds()  # (x_min, y_min, x_max, y_max)
        # 大范围地形自适应：step 和 goal 连接距离按地形尺寸缩放
        terrain_size = max(self.bounds[2] - self.bounds[0],
                           self.bounds[3] - self.bounds[1])
        if terrain_size > 5000:
            auto_step = terrain_size * 0.008  # ~0.8% 地形尺寸
            self.p.step = max(self.p.step, auto_step)
            # 转弯半径也适当放大（大范围用小转弯半径路径太碎）
            self.rho = max(self.rho, terrain_size * 0.003)
            # 碰撞检查步长：大范围放宽（网格分辨率粗，没必要每 8m 查一次）
            self._collision_step = max(8.0, terrain_size * 0.0005)  # ~0.05% 地形尺寸
            # 近邻数：大范围减少（降低 choose-parent/rewire 开销）
            self._n_neighbors = 6
            # 朝目标采样概率提高（大空间需要更聚焦）
            self.p.goal_bias = max(self.p.goal_bias, 0.15)
        else:
            self._collision_step = 8.0
            self._n_neighbors = 12

    # ------------------------------------------------------------------
    # 采样器（对应 TerrainStateSampler::sampleUniform）
    # ------------------------------------------------------------------
    def _sample(self) -> Optional[State]:
        x_min, y_min, x_max, y_max = self.bounds
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        elev = self.terrain.at_enu(x, y)
        if elev is None:
            return None
        z = elev + random.uniform(self.p.agl_min, self.p.agl_max)
        return (x, y, z, random.uniform(-math.pi, math.pi))

    # ------------------------------------------------------------------
    # 有效性（对应 TerrainValidityChecker，两层检查 + 禁飞区）
    # ------------------------------------------------------------------
    def _valid_state(self, s: State) -> bool:
        if self.terrain.is_in_collision(s[0], s[1], s[2], self.p.safety_margin):
            return False
        if self.terrain.above_ceiling(s[0], s[1], s[2], self.p.agl_max):
            return False
        for zone in self.p.nofly_zones:
            if TerrainAStarPlanner._point_in_polygon(s[0], s[1], zone):
                return False
        return True

    def _valid_edge(self, s_from: State, path: DubinsAirplanePath,
                    stop_len: Optional[float] = None) -> bool:
        for pt in walk_path(path, s_from, step=self._collision_step, stop_len=stop_len):
            if not self._valid_state(pt):
                return False
        return True

    # ------------------------------------------------------------------
    # 目标集（对应 setupProblem 的 loiter circle goal）
    # ------------------------------------------------------------------
    def make_goals(self, goal_enu: Tuple[float, float],
                   start_state: State) -> List[State]:
        gx, gy = goal_enu
        if self.p.loiter == 'none':
            yaw = math.atan2(gy - start_state[1], gx - start_state[0])
            return [self._goal_state(gx, gy, yaw)]

        direction = 1.0 if self.p.loiter == 'ccw' else -1.0
        goals = []
        for i in range(16):  # 悬停圆上 16 个切入点（对应 GoalStates 多目标）
            ang = i * 2.0 * math.pi / 16.0
            px = gx + self.rho * math.cos(ang)
            py = gy + self.rho * math.sin(ang)
            yaw = ang + direction * math.pi / 2.0  # 切向航向
            g = self._goal_state(px, py, yaw)
            if g is not None:
                goals.append(g)
        return goals or [self._goal_state(gx, gy, 0.0)]

    def _goal_state(self, x: float, y: float, yaw: float) -> Optional[State]:
        elev = self.terrain.at_enu(x, y)
        if elev is None:
            return None
        z = elev + self.p.agl_min
        s = (x, y, z, yaw)
        return s if self._valid_state(s) else None

    def _clamp_start(self, s: State) -> Optional[State]:
        elev = self.terrain.at_enu(s[0], s[1])
        if elev is None:
            return None
        z = min(max(elev + self.p.safety_margin + 1.0, s[2]),
                elev + self.p.agl_max)
        state = (s[0], s[1], z, s[3])
        return state if self._valid_state(state) else None

    # ------------------------------------------------------------------
    # RRT* 主循环（对应 TerrainOmplRrt::Solve，时间预算制）
    # ------------------------------------------------------------------
    def solve(self, start_state: State, goal_enu: Tuple[float, float],
              progress: Optional[dict] = None) -> Optional[_Node]:
        start = self._clamp_start(start_state)
        if start is None:
            raise ValueError("起点状态无效：落在障碍内/地形外，请换个位置")
        goals = self.make_goals(goal_enu, start)
        if not goals:
            raise ValueError("终点状态无效：落在障碍内/地形外，请换个位置")

        goal_pos = np.array([[g[0], g[1]] for g in goals])
        nodes: List[_Node] = [_Node(start)]
        node_xy = np.array([[start[0], start[1]]])

        best: Optional[_Node] = None       # 已连接到目标的末端树节点
        best_cost = math.inf
        t0 = time.time()
        deadline = t0 + self.p.time_budget
        iterations = 0

        while time.time() < deadline:
            iterations += 1
            # 1) 采样（含 goal bias）
            if random.random() < self.p.goal_bias:
                target = goals[random.randrange(len(goals))]
            else:
                target = self._sample()
                if target is None:
                    continue

            # 2) 找最近节点（欧氏预筛）并 steer
            d2 = ((node_xy - np.array(target[:2])) ** 2).sum(axis=1)
            order = np.argsort(d2)
            new_node = None
            for idx in order[:4]:
                near = nodes[idx]
                path = dubins_airplane(near.state, target, self.rho, self.gamma)
                if path is None:
                    continue
                L = path.length_xy()
                truncated = L > self.p.step
                stop = self.p.step if truncated else None
                if not self._valid_edge(near.state, path, stop_len=stop):
                    continue
                s_new = state_at(path, near.state, self.p.step) if truncated else target
                new_node = _Node(s_new, near, near.cost + min(L, self.p.step))
                break
            if new_node is None:
                continue

            # 3) choose parent：近邻里找代价更小的父节点
            d2n = ((node_xy - np.array(new_node.state[:2])) ** 2).sum(axis=1)
            near_idx = np.argsort(d2n)[:self._n_neighbors]
            for idx in near_idx:
                cand = nodes[idx]
                path = dubins_airplane(cand.state, new_node.state, self.rho, self.gamma)
                if path is None or path.length_xy() > 1.6 * self.p.step:
                    continue
                if not self._valid_edge(cand.state, path):
                    continue
                c = cand.cost + path.length_xy()
                if c < new_node.cost - 1e-6:
                    new_node.cost = c
                    new_node.parent = cand
            new_node.parent.children.append(new_node)
            nodes.append(new_node)
            node_xy = np.vstack([node_xy, [new_node.state[0], new_node.state[1]]])

            # 4) rewire：把近邻挂到新节点下若更优
            for idx in near_idx:
                cand = nodes[idx]
                if cand is new_node or cand is new_node.parent:
                    continue
                path = dubins_airplane(new_node.state, cand.state, self.rho, self.gamma)
                if path is None or path.length_xy() > 1.6 * self.p.step:
                    continue
                if not self._valid_edge(new_node.state, path):
                    continue
                c = new_node.cost + path.length_xy()
                if c < cand.cost - 1e-6:
                    self._reparent(cand, new_node, c)

            # 5) 连接目标集（Dubins-to-goal）
            if best_cost > 0.0:
                dg2 = ((goal_pos - np.array(new_node.state[:2])) ** 2).sum(axis=1)
                for gi in np.argsort(dg2)[:3]:
                    goal = goals[gi]
                    path = dubins_airplane(new_node.state, goal, self.rho, self.gamma)
                    if path is None or path.length_xy() > 3.0 * self.p.step:
                        continue
                    if not self._valid_edge(new_node.state, path):
                        continue
                    c = new_node.cost + path.length_xy()
                    if c < best_cost:
                        best_cost = c
                        best = new_node

            if progress is not None:
                progress.update(iterations=iterations,
                                best_length=round(best_cost, 1),
                                elapsed=round(time.time() - t0, 2))

        if best is None:
            return None
        # 回溯树链 + 末段目标状态
        states: List[State] = []
        cur = best
        while cur is not None:
            states.append(cur.state)
            cur = cur.parent
        states.reverse()
        states.append(self._closest_goal(best.state, goals))
        return self._build_result(states, iterations, time.time() - t0, best_cost)

    def _closest_goal(self, s: State, goals: List[State]) -> State:
        best_g, best_L = goals[0], math.inf
        for g in goals:
            path = dubins_airplane(s, g, self.rho, self.gamma)
            if path is None:
                continue
            L = path.length_xy()
            if L < best_L:
                best_g, best_L = g, L
        return best_g

    def _reparent(self, node: _Node, new_parent: _Node, new_cost: float) -> None:
        """把 node 挂到 new_parent 下并更新整棵子树代价。"""
        # 防环：new_parent 不能在 node 的子孙里
        cur = new_parent
        while cur is not None:
            if cur is node:
                return
            cur = cur.parent
        if node.parent is not None:
            node.parent.children.remove(node)
        node.parent = new_parent
        delta = new_cost - node.cost
        node.cost = new_cost
        new_parent.children.append(node)
        stack = [node]
        while stack:
            n = stack.pop()
            for ch in n.children:
                ch.cost += delta
                stack.append(ch)

    # ------------------------------------------------------------------
    # 求解后处理：地形越障高度重排（terrain clearance，非贴地跟随）
    # ------------------------------------------------------------------
    def _build_result(self, states: List[State], iterations: int,
                      plan_time: float, solution_cost: float) -> Plan3DResult:
        # 1) 沿整条 Dubins 链稠密采样（保留可飞的水平形状 + RRT* 高度）
        dense: List[Tuple[float, float, float]] = []  # (x, y, z_rrt)
        for i in range(len(states) - 1):
            path = dubins_airplane(states[i], states[i + 1], self.rho, self.gamma)
            if path is None:
                continue
            pts = walk_path(path, states[i], step=5.0)
            if dense:
                pts = pts[1:]  # 去重连接点
            dense.extend([(p[0], p[1], p[2]) for p in pts])
        if len(dense) < 2:
            raise ValueError("路径过短，无法规划（起终点太近）")

        # 2) 地形越障高度重排
        #    策略：以 RRT* 高度为基准（已在地形上方），只升不降——
        #    遇山爬越、遇谷保持高度，不强制贴地下降
        xy = np.array([(d[0], d[1]) for d in dense])
        z_rrt = np.array([d[2] for d in dense])
        n = len(xy)
        elev = np.array([self.terrain.at_enu(x, y) or 0.0 for x, y in xy])
        ds = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
        ds = np.clip(ds, 0.5, None)
        tan_g = math.tan(self.gamma)  # 最大爬升/下降角正切

        # 初始高度 = max(RRT* 高度, 地形 + 安全余量)
        # RRT* 高度已经在地形上方（采样时 z = elev + AGL），保留它
        z = np.maximum(z_rrt, elev + self.p.safety_margin)

        # 前向传播：限制爬升率（z[i] 不能比 z[i-1] 高太多）
        for _ in range(5):
            for i in range(1, n):
                max_climb = z[i - 1] + ds[i - 1] * tan_g
                if z[i] > max_climb:
                    z[i] = max_climb
            # 地面净空：不低于地形 + 安全余量
            z = np.maximum(z, elev + self.p.safety_margin)
            # 天花板：不高于地形 + AGL 上限
            z = np.minimum(z, elev + self.p.agl_max)

        # 后向传播：限制下降率（z[i] 不能比 z[i+1] 低太多 → 下降不能太陡）
        for _ in range(5):
            for i in range(n - 2, -1, -1):
                min_descent = z[i + 1] - ds[i] * tan_g
                if z[i] < min_descent:
                    z[i] = min_descent
            z = np.maximum(z, elev + self.p.safety_margin)
            z = np.minimum(z, elev + self.p.agl_max)

        # 轻度平滑（消除爬升率限制导致的锯齿，但不过度压平）
        w = 5
        if n > w:
            pad = np.pad(z, (w // 2, w // 2), mode='edge')
            smooth = np.convolve(pad, np.ones(w) / w, mode='valid')
            z[1:-1] = smooth[1:-1]
        # 平滑后再检查地面净空
        z = np.maximum(z, elev + self.p.safety_margin)
        z = np.minimum(z, elev + self.p.agl_max)

        # 3) 稀疏化输出（≥10m 一个点）并转经纬度
        keep = [0]
        for i in range(1, n):
            if np.hypot(*(xy[i] - xy[keep[-1]])) >= 10.0 or i == n - 1:
                keep.append(i)
        waypoints = []
        for i in keep:
            lng, lat = self.terrain.to_geo(xy[i][0], xy[i][1])
            waypoints.append((lng, lat, float(z[i])))

        # 4) 直线段输出（对齐 planner_msgs/PathSegment，机载端按 waypoint 跟踪）
        segments = []
        total = 0.0
        for a, b in zip(waypoints, waypoints[1:]):
            mx = (b[0] - a[0]) * self.terrain._m_per_deg_lng(self.terrain.enu_origin()[1])
            my = (b[1] - a[1]) * 111320.0
            mz = b[2] - a[2]
            seg_len = math.sqrt(mx * mx + my * my + mz * mz)
            if seg_len < 1e-6:
                continue
            segments.append({
                "segment_start": list(a),
                "segment_tangent": [mx / seg_len, my / seg_len, mz / seg_len],
                "curvature": 0.0,
                "segment_length": seg_len,
                "is_periodic": False,
                "reached": False,
            })
            total += seg_len
        return Plan3DResult(waypoints=waypoints, segments=segments,
                            length=total, iterations=iterations,
                            plan_time=plan_time, solution_cost=solution_cost)
