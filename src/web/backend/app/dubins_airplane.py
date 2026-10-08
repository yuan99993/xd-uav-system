"""(Non-optimal) Dubins Airplane 状态空间的纯 Python 移植。

移植自 terrain-navigation：
    terrain_planner/include/terrain_planner/DubinsAirplane.hpp
    terrain_planner/src/DubinsAirplane.cpp
    terrain_planner/include/terrain_planner/DubinsPath.hpp
    terrain_planner/src/DubinsPath.cpp

为什么需要它：
    固定翼不能原地转弯/爬升，A* 折线对它不可飞。
    Dubins Airplane 在 2D Dubins（最小转弯半径约束）基础上加了
    爬升角约束（gamma_max），路径 = [起始螺旋圈?] + 2D Dubins + [结束螺旋圈?]，
    z 沿路径以恒定爬升角 gamma 线性变化，保证终点高度精确到达。

状态定义（与原项目一致，ENU 米制）：
    (x, y, z, yaw)   yaw 弧度，逆时针为正（数学约定，东=0）
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

TWO_PI = 2.0 * math.pi


def mod2pi(theta: float) -> float:
    """归一化到 [0, 2π)，对应原项目 ompl::base::mod2pi。"""
    v = math.fmod(theta, TWO_PI)
    if v < 0:
        v += TWO_PI
    return v


# ---------------------------------------------------------------------------
# 2D Dubins 六种 word 的解析解（长度按转弯半径=1 归一化，d 也归一化）。
# 公式与 OMPL dubins.cpp / 原项目 t_lsl..q_lrl 系列一致（Shkel & Lumelsky 2001）。
# 返回 (t, p, q)，None 表示该 word 无解。
# ---------------------------------------------------------------------------
def _lsl(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    p_sq = 2 + d * d - 2 * (ca * cb + sa * sb) + 2 * d * (sa - sb)
    if p_sq < 0:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    return (mod2pi(tmp - alpha), math.sqrt(p_sq), mod2pi(beta - tmp))


def _rsr(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    p_sq = 2 + d * d - 2 * (ca * cb + sa * sb) + 2 * d * (sb - sa)
    if p_sq < 0:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    return (mod2pi(alpha - tmp), math.sqrt(p_sq), mod2pi(tmp - beta))


def _lsr(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    p_sq = -2 + d * d + 2 * (ca * cb + sa * sb) + 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return (mod2pi(tmp - alpha), p, mod2pi(tmp - mod2pi(beta)))


def _rsl(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    p_sq = -2 + d * d + 2 * (ca * cb + sa * sb) - 2 * d * (sa + sb)
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    theta = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return (mod2pi(alpha - theta), p, mod2pi(beta - theta))


def _rlr(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    tmp = (6.0 - d * d + 2 * (ca * cb + sa * sb) + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1:
        return None
    p = mod2pi(TWO_PI - math.acos(tmp))
    phi = math.atan2(ca - cb, d - sa + sb)
    t = mod2pi(alpha - phi + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return (t, p, q)


def _lrl(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    tmp = (6.0 - d * d + 2 * (ca * cb + sa * sb) - 2 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1:
        return None
    p = mod2pi(TWO_PI - math.acos(tmp))
    theta = math.atan2(cb - ca, d + sa - sb)
    t = mod2pi(-alpha + theta + 0.5 * p)
    q = mod2pi(beta - alpha - t + p)
    return (t, p, q)


# 顺序对齐原项目 Index 枚举：TYPE_LSL=0, RSR=1, RSL=2, LSR=3, RLR=4, LRL=5
_WORD_SOLVERS = [_lsl, _rsr, _rsl, _lsr, _rlr, _lrl]
_WORD_NAMES = ["LSL", "RSR", "RSL", "LSR", "RLR", "LRL"]


class DubinsAirplanePath:
    """一条 (non-optimal) Dubins airplane 路径。

    对应原项目 DubinsPath：6 个段（长度为 xy 平面投影长度，米）：
        [0] 起始螺旋（k 整圈，半径 rho*ratio）  [1] 2D 段1  [2] 中间机动（本实现恒为 0）
        [3] 2D 段2  [4] 2D 段3  [5] 结束螺旋
    gamma：全程恒定爬升角（弧度，可负），z 沿 xy 路径长度以 tan(gamma) 线性变化。
    """

    __slots__ = ("idx", "word", "lengths", "gamma", "radius_ratio", "rho", "gamma_max")

    def __init__(self, idx: int, word: str, lengths: List[float], rho: float,
                 gamma: float, radius_ratio: Optional[List[float]] = None,
                 gamma_max: float = 0.0):
        self.idx = idx
        self.word = word                      # e.g. "LSL"，三个 2D 段的转向
        self.lengths = lengths                # 6 段 xy 投影长度（米）
        self.gamma = gamma
        self.rho = rho
        self.radius_ratio = radius_ratio or [1.0] * 6
        self.gamma_max = gamma_max

    def length_xy(self) -> float:
        return sum(self.lengths)

    def length_3d(self) -> float:
        """空间斜距 = xy 投影长度 / cos(|gamma|)（对应 length_3D）。"""
        return self.length_xy() / max(1e-12, math.cos(abs(self.gamma)))

    def climb_rate(self) -> float:
        return math.tan(self.gamma)


def _solve_2d(alpha: float, beta: float, d: float) -> Optional[Tuple[int, str, float, float, float]]:
    """解所有 6 种 word（对应 calcDubPathWithoutClassification），返回最短可行解。"""
    best = None
    for idx, solver in enumerate(_WORD_SOLVERS):
        r = solver(alpha, beta, d)
        if r is None:
            continue
        t, p, q = r
        if best is None or t + p + q < best[2] + best[3] + best[4]:
            best = (idx, _WORD_NAMES[idx], t, p, q)
    return best


def compute_opt_radius_ratio(fabs_dz: float, L: float, tan_gamma: float, k: int) -> float:
    """对应 DubinsAirplane.cpp::computeOptRratio（螺旋放大半径使高度差精确到达）。"""
    return (fabs_dz - L * tan_gamma) / (TWO_PI * tan_gamma * k)


def dubins_airplane(s1: Tuple[float, float, float, float],
                    s2: Tuple[float, float, float, float],
                    rho: float = 66.66667,
                    gamma_max: float = 0.15) -> Optional[DubinsAirplanePath]:
    """计算 s1 → s2 的 (non-optimal) Dubins airplane 路径。

    对应 DubinsAirplane.cpp::dubins()（optimalStSp_=false 分支）。
    s = (x, y, z, yaw)，ENU 米。失败（重合/无解）返回 None。
    """
    x1, y1, z1, th1 = s1
    x2, y2, z2, th2 = s2

    dx = (x2 - x1) / rho
    dy = (y2 - y1) / rho
    dz = (z2 - z1) / rho
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return None  # 水平重合：无法构造有意义的路径（OMPL 里此情形同样退化）

    th = math.atan2(dy, dx)
    alpha = mod2pi(th1 - th)
    beta = mod2pi(th2 - th)

    best = _solve_2d(alpha, beta, d)
    if best is None:
        return None
    idx, word, t, p, q = best

    L = t + p + q                       # 归一化 2D 长度
    fabs_dz = abs(dz)
    tan_gmax = math.tan(gamma_max)
    tan_gmax_inv = 1.0 / tan_gmax
    lengths = [0.0, t * rho, 0.0, p * rho, q * rho, 0.0]
    ratio = [1.0] * 6

    if fabs_dz * tan_gmax_inv <= L:
        # 低高度情形：2D 路径长度足够消化高度差，gamma = atan2(dz, L)
        # （全部用归一化量，比值与米制等价）
        gamma = math.atan2(dz, L)
    elif fabs_dz * tan_gmax_inv >= L + TWO_PI:
        # 高高度情形：爬升角拉满 + 起始(爬)/结束(降)k整圈螺旋，
        # 螺旋半径放大到刚好把剩余高度差消化掉（computeOptRratio）。
        k = int(math.floor((fabs_dz * tan_gmax_inv - L) / TWO_PI))
        r = compute_opt_radius_ratio(fabs_dz, L, tan_gmax, k)  # 归一化半径比
        if dz >= 0:
            ratio[0] = r
            lengths[0] = k * TWO_PI * r * rho
            gamma = gamma_max
        else:
            ratio[5] = r
            lengths[5] = k * TWO_PI * r * rho
            gamma = -gamma_max
    else:
        # 中间高度情形（非最优解）：多飞至多一圈螺旋（半径=最小转弯半径），
        # gamma = atan2(dz, L + 螺旋长度)，精确到达。
        k = int(math.floor((fabs_dz * tan_gmax_inv - L) / TWO_PI)) + 1
        if dz >= 0:
            lengths[0] = k * TWO_PI * rho
            gamma = math.atan2(dz, L + k * TWO_PI)
        else:
            lengths[5] = k * TWO_PI * rho
            gamma = math.atan2(dz, L + k * TWO_PI)

    return DubinsAirplanePath(idx, word, lengths, rho, gamma, ratio, gamma_max)


# ---------------------------------------------------------------------------
# 路径上取点 / 截断（对应 calculateSegmentStarts / interpolate）
# ---------------------------------------------------------------------------
def _primitives(path: DubinsAirplanePath) -> List[Tuple[str, float, float]]:
    """把 6 段展开成 (kind, length_m, radius) 列表，kind ∈ {'L','R','S'}。

    螺旋段方向与相邻 2D 段一致（对应 convert_idx）：
        段0/1/2 → word[0]，段3 → word[1]，段4/5 → word[2]。
    """
    w = path.word
    out = []
    for i, length in enumerate(path.lengths):
        if length <= 1e-12:
            continue
        radius = path.rho * path.radius_ratio[i]
        if i == 0:
            kind = w[0]
        elif i == 1 or i == 2:
            kind = w[0]
        elif i == 3:
            kind = w[1]
        else:
            kind = w[2]
        if kind == "S":
            radius = 0.0
        out.append((kind, length, radius))
    return out


def _advance(pose, kind: str, length: float, radius: float, tan_g: float):
    """从 pose=(x,y,z,yaw) 前进 length（xy 投影米），返回新 pose。"""
    x, y, z, yaw = pose
    if kind == "S":
        x += length * math.cos(yaw)
        y += length * math.sin(yaw)
    elif kind == "L":
        dphi = length / radius
        x += radius * (math.sin(yaw + dphi) - math.sin(yaw))
        y += radius * (math.cos(yaw) - math.cos(yaw + dphi))
        yaw += dphi
    else:  # 'R'
        dphi = length / radius
        x += radius * (math.sin(yaw) - math.sin(yaw - dphi))
        y += radius * (math.cos(yaw - dphi) - math.cos(yaw))
        yaw -= dphi
    z += length * tan_g
    return (x, y, z, yaw)


def walk_path(path: DubinsAirplanePath, s_from, step: float = 5.0,
              stop_len: Optional[float] = None) -> List[Tuple[float, float, float, float]]:
    """沿路径每 step（xy 米）取一个点，含终点，返回 [(x,y,z,yaw), ...]。

    z 沿 xy 长度以 tan(gamma) 线性变化（对应 interpolate 的 z 处理）。
    stop_len：只走前 stop_len 米（RRT* steer 截断用），None 表示走完整条路径。
    """
    pose = tuple(s_from)
    tan_g = math.tan(path.gamma)
    pts = [pose]
    cum = 0.0
    for kind, length, radius in _primitives(path):
        n = max(1, int(math.ceil(length / step)))
        ds = length / n
        for _ in range(n):
            pose = _advance(pose, kind, ds, radius, tan_g)
            cum += ds
            pts.append(pose)
            if stop_len is not None and cum >= stop_len:
                return pts
    return pts


def state_at(path: DubinsAirplanePath, s_from, target_len: float):
    """取路径上 xy 投影长度 = target_len 处的状态（用于 steer 截断）。"""
    pose = tuple(s_from)
    tan_g = math.tan(path.gamma)
    remaining = target_len
    for kind, length, radius in _primitives(path):
        if remaining <= length:
            return _advance(pose, kind, remaining, radius, tan_g)
        pose = _advance(pose, kind, length, radius, tan_g)
        remaining -= length
    return pose


def path_to_points(path: DubinsAirplanePath, s_from, step: float = 5.0):
    """walk_path 的别名（语义化封装，供规划器调用）。"""
    return walk_path(path, s_from, step)
