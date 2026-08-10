"""
FeasibilityLimiter — 可行性约束限幅器。

受 ego-planner-swarm 的 feasibility 约束启发:
- 参考 max_vel / max_acc / max_jerk 物理限制
- 对 Follower 输出的速度指令进行动态可行性检查
- 当指令超限时自动缩放或截断
- 维护指令的时间一致性避免突变

核心参考:
  ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp
  - velocity feasibility: 检查 ||v|| <= max_vel
  - acceleration feasibility: 检查 ||a|| <= max_acc
  - 使用 feasibility_tolerance_ 比例容许轻微超限
"""

import time
import math
from typing import Tuple, Optional
from collections import deque
import logging

logger = logging.getLogger(__name__)


class FeasibilityLimiter:
    """
    动态可行性约束限幅器。

    检查 Follower 输出的速度/加速度/加加速度指令是否在物理限制内，
    对超限指令进行平滑限幅处理。

    三层约束（参考 ego-planner）:
    1. 速度幅值: ||v|| <= max_vel
    2. 加速度幅值: ||a|| <= max_acc (从速度变化推算)
    3. 加加速度幅值: ||j|| <= max_jerk (从加速度变化推算)
    """

    def __init__(
        self,
        max_vel: float = 8.0,           # m/s, 最大速度
        max_acc: float = 5.0,           # m/s², 最大加速度
        max_jerk: float = 20.0,         # m/s³, 最大加加速度
        feasibility_tolerance: float = 0.1,  # 容许超限比例 (ego-planner 默认)
        history_size: int = 5,          # 指令历史
    ):
        """
        初始化可行性限幅器。

        Args:
            max_vel: 最大合速度 (m/s)
            max_acc: 最大合加速度 (m/s²)
            max_jerk: 最大合加加速度 (m/s³)
            feasibility_tolerance: 容许超限比例 [0, 1]
            history_size: 指令历史缓存大小
        """
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_jerk = max_jerk
        self.feasibility_tolerance = feasibility_tolerance
        self.tol_factor = 1.0 + feasibility_tolerance

        # 指令历史
        self.vel_history: deque = deque(maxlen=history_size)
        self.acc_history: deque = deque(maxlen=history_size)
        self.last_time = 0.0

        # 统计
        self.vel_clamp_count = 0
        self.acc_clamp_count = 0
        self.jerk_clamp_count = 0

        logger.info(
            "[FeasibilityLimiter] Initialized: max_vel=%.1f m/s, max_acc=%.1f m/s², "
            "max_jerk=%.1f m/s³, tol=%.1f%%",
            max_vel, max_acc, max_jerk, feasibility_tolerance * 100,
        )

    def limit(
        self,
        vx: float, vy: float, vz: float,
        timestamp: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        对速度指令进行可行性约束。

        Args:
            vx, vy, vz: 期望速度 (m/s), body frame
            timestamp: 当前时间戳

        Returns:
            (vx_clamped, vy_clamped, vz_clamped) 约束后速度
        """
        if timestamp is None:
            timestamp = time.time()

        dt = timestamp - self.last_time if self.last_time > 0 else 0.033
        self.last_time = timestamp

        # ── 1. 速度幅值约束 ──────────────────────────────────────────────
        vel_mag = math.sqrt(vx * vx + vy * vy + vz * vz)
        if vel_mag > self.max_vel * self.tol_factor:
            scale = (self.max_vel * self.tol_factor) / vel_mag
            vx *= scale
            vy *= scale
            vz *= scale
            self.vel_clamp_count += 1
            logger.debug(
                "[FeasibilityLimiter] Velocity clamped: %.2f → %.2f m/s",
                vel_mag, vel_mag * scale,
            )

        # ── 2. 加速度约束 ────────────────────────────────────────────────
        if dt > 0 and len(self.vel_history) > 0:
            prev_vx, prev_vy, prev_vz = self.vel_history[-1]
            ax = (vx - prev_vx) / dt
            ay = (vy - prev_vy) / dt
            az = (vz - prev_vz) / dt
            acc_mag = math.sqrt(ax * ax + ay * ay + az * az)

            if acc_mag > self.max_acc * self.tol_factor:
                # 缩放加速度 → 反向计算新的速度
                scale = (self.max_acc * self.tol_factor) / acc_mag
                ax *= scale
                ay *= scale
                az *= scale
                vx = prev_vx + ax * dt
                vy = prev_vy + ay * dt
                vz = prev_vz + az * dt
                self.acc_clamp_count += 1
                logger.debug(
                    "[FeasibilityLimiter] Acceleration clamped: %.2f → %.2f m/s²",
                    acc_mag, acc_mag * scale,
                )

        # ── 3. 加加速度约束 ──────────────────────────────────────────────
        if dt > 0 and len(self.acc_history) > 0:
            prev_ax, prev_ay, prev_az = self.acc_history[-1]
            if len(self.vel_history) > 0:
                pvx, pvy, pvz = self.vel_history[-1]
                ax = (vx - pvx) / dt
                ay = (vy - pvy) / dt
                az = (vz - pvz) / dt
                jx = (ax - prev_ax) / dt
                jy = (ay - prev_ay) / dt
                jz = (az - prev_az) / dt
                jerk_mag = math.sqrt(jx * jx + jy * jy + jz * jz)

                if jerk_mag > self.max_jerk * self.tol_factor:
                    scale = (self.max_jerk * self.tol_factor) / jerk_mag
                    jx *= scale
                    jy *= scale
                    jz *= scale
                    ax = prev_ax + jx * dt
                    ay = prev_ay + jy * dt
                    az = prev_az + jz * dt
                    if len(self.vel_history) > 0:
                        pvx, pvy, pvz = self.vel_history[-1]
                        vx = pvx + ax * dt
                        vy = pvy + ay * dt
                        vz = pvz + az * dt
                    self.jerk_clamp_count += 1

        # ── 存储历史 ─────────────────────────────────────────────────────
        self.vel_history.append((vx, vy, vz))
        if dt > 0 and len(self.vel_history) >= 2:
            pvx, pvy, pvz = self.vel_history[-2]
            ax = (vx - pvx) / dt
            ay = (vy - pvy) / dt
            az = (vz - pvz) / dt
            self.acc_history.append((ax, ay, az))

        return (vx, vy, vz)

    def get_stats(self) -> dict:
        """获取限幅统计信息。"""
        return {
            'vel_clamp_count': self.vel_clamp_count,
            'acc_clamp_count': self.acc_clamp_count,
            'jerk_clamp_count': self.jerk_clamp_count,
            'max_vel': self.max_vel,
            'max_acc': self.max_acc,
            'max_jerk': self.max_jerk,
        }

    def reset(self):
        """重置限幅器状态。"""
        self.vel_history.clear()
        self.acc_history.clear()
        self.last_time = 0.0
        self.vel_clamp_count = 0
        self.acc_clamp_count = 0
        self.jerk_clamp_count = 0
