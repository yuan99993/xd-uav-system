"""
JerkLimitedSmoother — 加加速度限制平滑器。

受 ego-planner-swarm 的轨迹平滑策略启发:
- 对 Follower 输出的 yaw rate 和 velocity 指令进行 jerk-limited 平滑
- 参考 ego-planner 中 B-spline 轨迹的 jerk 约束
- 提供比简单 EMA 更平滑的指令过渡

核心思想:
- ego-planner 使用 B-spline 的3阶连续性保证 jerk 有界
- 在离散指令层面，通过限制相邻帧之间的加速度变化率来模拟 jerk 约束
"""

import math
from typing import Optional
from collections import deque
import logging

logger = logging.getLogger(__name__)


class JerkLimitedSmoother:
    """
    加加速度限制平滑器。

    对标量或向量指令应用 jerk-limited 平滑，
    保证指令的加速度变化率不超过预设阈值。

    参考 ego-planner 的 feasibility 约束:
    - 速度 → 加速度 → 加加速度 三级约束链
    """

    def __init__(
        self,
        max_jerk: float = 20.0,          # 最大加加速度 (units/s³)
        smoothing_window: int = 3,       # 平滑窗口
        enabled: bool = True,
    ):
        """
        初始化加加速度限制平滑器。

        Args:
            max_jerk: 最大加加速度幅值
            smoothing_window: 用于加加速度计算的窗口大小
            enabled: 是否启用
        """
        self.max_jerk = max_jerk
        self.smoothing_window = smoothing_window
        self.enabled = enabled

        # 状态
        self.prev_value = 0.0
        self.prev_velocity = 0.0     # 一阶导 (rate of change)
        self.prev_acceleration = 0.0 # 二阶导 (acceleration)
        self.prev_dt = 0.033

        # 多轴状态
        self.prev_values_3d = (0.0, 0.0, 0.0)
        self.prev_velocities_3d = (0.0, 0.0, 0.0)

        # 历史
        self.value_history: deque = deque(maxlen=smoothing_window + 2)

    def smooth_scalar(self, value: float, dt: float) -> float:
        """
        对标量值进行 jerk-limited 平滑。

        Args:
            value: 目标值
            dt: 时间步长 (s)

        Returns:
            平滑后的值
        """
        if not self.enabled or dt <= 0:
            self.prev_value = value
            return value

        # 计算当前速度（一阶导）
        current_velocity = (value - self.prev_value) / dt

        # 计算当前加速度（二阶导）
        current_acceleration = (current_velocity - self.prev_velocity) / dt

        # 计算加加速度（三阶导）
        current_jerk = (current_acceleration - self.prev_acceleration) / dt

        # Jerk 限幅
        if abs(current_jerk) > self.max_jerk:
            # 限制 jerk → 反推允许的加速度变化
            clamped_jerk = math.copysign(self.max_jerk, current_jerk)
            allowed_acc = self.prev_acceleration + clamped_jerk * dt

            # 反推允许的速度
            allowed_velocity = self.prev_velocity + allowed_acc * dt

            # 反推允许的值
            smoothed_value = self.prev_value + allowed_velocity * dt
        else:
            smoothed_value = value

        # 更新状态
        self.prev_acceleration = (smoothed_value - self.prev_value) / dt - self.prev_velocity
        self.prev_acceleration = (self.prev_acceleration + 
            (smoothed_value - self.prev_value) / dt) / dt
        # 简化: 使用 backward difference
        new_velocity = (smoothed_value - self.prev_value) / dt
        self.prev_acceleration = (new_velocity - self.prev_velocity) / dt
        self.prev_velocity = new_velocity
        self.prev_value = smoothed_value
        self.prev_dt = dt

        self.value_history.append(smoothed_value)
        return smoothed_value

    def smooth_3d(
        self, vx: float, vy: float, vz: float, dt: float,
    ) -> tuple:
        """
        对3D向量进行 jerk-limited 平滑。

        Args:
            vx, vy, vz: 目标值
            dt: 时间步长

        Returns:
            (sx, sy, sz) 平滑后值
        """
        if not self.enabled or dt <= 0:
            self.prev_values_3d = (vx, vy, vz)
            return (vx, vy, vz)

        px, py, pz = self.prev_values_3d

        # 计算速度
        cvx = (vx - px) / dt
        cvy = (vy - py) / dt
        cvz = (vz - pz) / dt

        # 计算加速度
        pvx, pvy, pvz = self.prev_velocities_3d
        cax = (cvx - pvx) / dt
        cay = (cvy - pvy) / dt
        caz = (cvz - pvz) / dt

        # 计算 jerk 幅值
        # 简化: 直接对每个轴分别限幅
        def _clamp_jerk(j: float, max_j: float) -> float:
            if abs(j) > max_j:
                return math.copysign(max_j, j)
            return j

        cax_allowed = _clamp_jerk(cax, self.max_jerk)
        cay_allowed = _clamp_jerk(cay, self.max_jerk)
        caz_allowed = _clamp_jerk(caz, self.max_jerk)

        # 检查是否有轴被限幅
        if (cax_allowed != cax or cay_allowed != cay or caz_allowed != caz):
            # 反推允许的速度
            avx = pvx + cax_allowed * dt
            avy = pvy + cay_allowed * dt
            avz = pvz + caz_allowed * dt

            # 反推允许的值
            sx = px + avx * dt
            sy = py + avy * dt
            sz = pz + avz * dt
        else:
            sx, sy, sz = vx, vy, vz

        # 更新状态
        nvx = (sx - px) / dt
        nvy = (sy - py) / dt
        nvz = (sz - pz) / dt
        self.prev_acceleration = math.sqrt(
            ((nvx - pvx) / dt) ** 2 +
            ((nvy - pvy) / dt) ** 2 +
            ((nvz - pvz) / dt) ** 2
        )
        self.prev_velocities_3d = (nvx, nvy, nvz)
        self.prev_values_3d = (sx, sy, sz)
        self.prev_dt = dt

        return (sx, sy, sz)

    def reset(self):
        """重置平滑器状态。"""
        self.prev_value = 0.0
        self.prev_velocity = 0.0
        self.prev_acceleration = 0.0
        self.prev_dt = 0.033
        self.prev_values_3d = (0.0, 0.0, 0.0)
        self.prev_velocities_3d = (0.0, 0.0, 0.0)
        self.value_history.clear()
