"""
BSplinePredictor — B样条轨迹预测器。

受 ego-planner-swarm 的 UniformBspline 启发:
- 使用 B-spline 参数化目标运动轨迹（3阶，clamped）
- 提供平滑的位置/速度/加速度预测
- 内建速度/加速度可行性约束（max_vel, max_acc）
- 支持从历史检测点拟合控制点

与 MotionPredictor (EMA线性预测) 互补:
- MotionPredictor: 短期遮挡预测（1-5帧）
- BSplinePredictor: 中长期轨迹预测和平滑

核心算法参考:
  ego-planner-swarm/src/planner/bspline_opt/src/uniform_bspline.cpp
"""

import numpy as np
import time
from collections import deque
from typing import Optional, Tuple, List
import logging

logger = logging.getLogger(__name__)


class BSplinePredictor:
    """
    3阶均匀 B-spline 轨迹预测器。

    从目标的历史检测位置拟合 B-spline 控制点，
    提供平滑的位置/速度/加速度多步预测。

    核心约束（参考 ego-planner）:
    - max_vel: 最大预测速度 (px/s)
    - max_acc: 最大预测加速度 (px/s²)
    - 控制点间距: dt * velocity_estimate
    """

    def __init__(
        self,
        order: int = 3,                      # B-spline 阶数
        num_control_points: int = 8,          # 控制点数量
        history_size: int = 15,              # 历史点缓存大小
        max_vel: float = 500.0,              # px/s, 最大目标速度
        max_acc: float = 200.0,              # px/s², 最大目标加速度
        dt_estimate: float = 1.0 / 30.0,     # 时间步长估计 (s)
        prediction_horizon: float = 1.0,     # 预测时长 (s)
    ):
        """
        初始化 B-spline 预测器。

        Args:
            order: B-spline 阶数 (通常3阶)
            num_control_points: 控制点数量
            history_size: 历史位置缓存大小
            max_vel: 最大允许速度 (px/s)
            max_acc: 最大允许加速度 (px/s²)
            dt_estimate: 估计的时间步长
            prediction_horizon: 预测时长
        """
        self.order = order
        self.num_control_points = num_control_points
        self.history_size = history_size
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.dt_estimate = dt_estimate
        self.prediction_horizon = prediction_horizon

        # 历史记录: deque of (timestamp, x, y)
        self.position_history: deque = deque(maxlen=history_size)
        self.last_update_time = 0.0

        # B-spline 控制点 (2, N) — 当前拟合结果
        self.control_points: Optional[np.ndarray] = None
        self.knot_span = dt_estimate
        self.fitted = False

        # 基函数预计算
        self._basis_cache: dict = {}

        logger.info(
            "[BSplinePredictor] Initialized: order=%d, N=%d, max_vel=%.0f px/s, "
            "max_acc=%.0f px/s², horizon=%.1f s",
            order, num_control_points, max_vel, max_acc, prediction_horizon,
        )

    def _get_uniform_knots(self, n: int, dt: float) -> np.ndarray:
        """
        生成 clamped 均匀 B-spline 节点向量。

        Args:
            n: 控制点数量
            dt: 节点间隔

        Returns:
            节点向量 (n + order + 1,)
        """
        m = n + self.order + 1
        knots = np.zeros(m)
        # Clamped: 首尾各 order+1 个重复节点
        knots[:self.order] = 0.0
        for i in range(self.order, n + 1):
            knots[i] = (i - self.order) * dt
        knots[n + 1:] = (n - self.order) * dt
        return knots

    def _basis_function(self, i: int, k: int, t: float, knots: np.ndarray) -> float:
        """
        Cox-de Boor 递推计算 B-spline 基函数值。

        Args:
            i: 控制点索引
            k: 当前阶数
            t: 参数值
            knots: 节点向量

        Returns:
            基函数值 N_{i,k}(t)
        """
        if k == 0:
            return 1.0 if knots[i] <= t < knots[i + 1] else 0.0

        d1 = knots[i + k] - knots[i]
        d2 = knots[i + k + 1] - knots[i + 1]

        c1 = 0.0
        if d1 > 1e-10:
            c1 = (t - knots[i]) / d1 * self._basis_function(i, k - 1, t, knots)

        c2 = 0.0
        if d2 > 1e-10:
            c2 = (knots[i + k + 1] - t) / d2 * self._basis_function(i + 1, k - 1, t, knots)

        return c1 + c2

    def _evaluate_bspline(self, t: float) -> Tuple[float, float]:
        """
        在参数 t 处评估 B-spline 曲线。

        Args:
            t: 参数值 [0, T_max]

        Returns:
            (x, y) 位置
        """
        if self.control_points is None:
            return (0.0, 0.0)

        n = self.control_points.shape[1]
        knots = self._get_uniform_knots(n, self.knot_span)

        x, y = 0.0, 0.0
        for i in range(n):
            b = self._basis_function(i, self.order, t, knots)
            x += self.control_points[0, i] * b
            y += self.control_points[1, i] * b

        return (x, y)

    def _evaluate_velocity(self, t: float) -> Tuple[float, float]:
        """
        在参数 t 处评估 B-spline 速度。

        对于3阶B-spline: 速度 = 控制点一阶差分的加权和
        """
        if self.control_points is None:
            return (0.0, 0.0)

        n = self.control_points.shape[1]
        if n < 2:
            return (0.0, 0.0)

        knots = self._get_uniform_knots(n, self.knot_span)

        vx, vy = 0.0, 0.0
        for i in range(n - 1):
            dcx = self.control_points[0, i + 1] - self.control_points[0, i]
            dcy = self.control_points[1, i + 1] - self.control_points[1, i]
            b = self._basis_function(i + 1, self.order - 1, t, knots)
            vx += self.order * dcx / self.knot_span * b
            vy += self.order * dcy / self.knot_span * b

        return (vx, vy)

    def update(self, position: Tuple[float, float], timestamp: float):
        """
        添加新的位置测量。

        Args:
            position: (x, y) 像素坐标
            timestamp: 时间戳
        """
        self.position_history.append((timestamp, position[0], position[1]))
        self.last_update_time = timestamp

        # 当有足够数据时重新拟合
        if len(self.position_history) >= self.order + 2:
            self._fit_bspline()

    def _fit_bspline(self):
        """
        从历史数据拟合 B-spline 控制点。

        方法: 最小二乘法拟合
        - 从历史点计算时间参数化
        - 构造基函数矩阵
        - 求解控制点
        - 应用可行性约束
        """
        history = list(self.position_history)
        if len(history) < self.order + 2:
            return

        n_hist = len(history)

        # 时间参数化 (弦长参数化)
        t0 = history[0][0]
        times = np.array([h[0] - t0 for h in history])
        total_time = times[-1] if times[-1] > 0 else 1.0

        # 控制点数量 (不超过历史点数)
        n_ctrl = min(self.num_control_points, n_hist - self.order)
        if n_ctrl < self.order + 1:
            return

        self.knot_span = total_time / (n_ctrl - self.order)
        knots = self._get_uniform_knots(n_ctrl, self.knot_span)

        # 提取位置
        px = np.array([h[1] for h in history])
        py = np.array([h[2] for h in history])

        # 构造基函数矩阵 B (n_hist x n_ctrl)
        B = np.zeros((n_hist, n_ctrl))
        for i in range(n_hist):
            t = times[i]
            # Clamp t 到有效范围
            t_clamped = max(knots[self.order], min(t, knots[n_ctrl]))
            for j in range(n_ctrl):
                B[i, j] = self._basis_function(j, self.order, t_clamped, knots)

        # 最小二乘求解: B^T B cp = B^T p
        try:
            BTB = B.T @ B + 1e-4 * np.eye(n_ctrl)  # 正则化
            BTpx = B.T @ px
            BTpy = B.T @ py

            cp_x = np.linalg.solve(BTB, BTpx)
            cp_y = np.linalg.solve(BTB, BTpy)
        except np.linalg.LinAlgError:
            logger.warning("[BSplinePredictor] Singular matrix in LS fit, skipping")
            return

        self.control_points = np.vstack([cp_x, cp_y])
        self.fitted = True

        # 可行性约束检查 (参考 ego-planner 的 feasibility_tolerance)
        self._enforce_feasibility()

    def _enforce_feasibility(self):
        """
        强制可行性约束（参考 ego-planner 的 max_vel/max_acc）。

        检查控制点间距对应的速度和加速度是否超限，
        若超限则均匀缩放控制点间距。
        """
        if self.control_points is None:
            return

        n = self.control_points.shape[1]
        dt = self.knot_span
        if dt <= 0:
            return

        # 检查速度
        max_cp_vel = 0.0
        for i in range(n - 1):
            dx = self.control_points[0, i + 1] - self.control_points[0, i]
            dy = self.control_points[1, i + 1] - self.control_points[1, i]
            vel = np.sqrt(dx * dx + dy * dy) / dt
            max_cp_vel = max(max_cp_vel, vel)

        if max_cp_vel > self.max_vel:
            scale = self.max_vel / max_cp_vel
            # 缩放控制点（保持起点不变）
            cp0 = self.control_points[:, 0:1]
            self.control_points = cp0 + scale * (self.control_points - cp0)
            logger.debug(
                "[BSplinePredictor] Velocity scaled by %.3f (%.0f → %.0f px/s)",
                scale, max_cp_vel, max_cp_vel * scale,
            )

        # 检查加速度
        if n >= 3:
            max_cp_acc = 0.0
            for i in range(n - 2):
                vx1 = (self.control_points[0, i + 1] - self.control_points[0, i]) / dt
                vy1 = (self.control_points[1, i + 1] - self.control_points[1, i]) / dt
                vx2 = (self.control_points[0, i + 2] - self.control_points[0, i + 1]) / dt
                vy2 = (self.control_points[1, i + 2] - self.control_points[1, i + 1]) / dt
                ax = (vx2 - vx1) / dt
                ay = (vy2 - vy1) / dt
                acc = np.sqrt(ax * ax + ay * ay)
                max_cp_acc = max(max_cp_acc, acc)

            if max_cp_acc > self.max_acc:
                # 如果加速度超限，增加 knot_span 等效于放慢轨迹
                scale = np.sqrt(self.max_acc / max_cp_acc)
                self.knot_span /= scale
                logger.debug(
                    "[BSplinePredictor] Acceleration scaled: dt %.4f → %.4f",
                    dt, self.knot_span,
                )

    def predict(self, t_future: float) -> Tuple[float, float]:
        """
        预测未来 t_future 秒后的位置。

        Args:
            t_future: 未来时间偏移 (s)

        Returns:
            (x, y) 预测位置
        """
        if not self.fitted or self.control_points is None:
            # 回退到最后一帧位置
            if self.position_history:
                last = self.position_history[-1]
                return (last[1], last[2])
            return (0.0, 0.0)

        n = self.control_points.shape[1]
        t_max = (n - self.order) * self.knot_span
        t = min(t_future, t_max * 0.95)  # 不超出范围
        return self._evaluate_bspline(t)

    def predict_velocity(self, t_future: float) -> Tuple[float, float]:
        """
        预测未来 t_future 秒后的速度。

        Args:
            t_future: 未来时间偏移 (s)

        Returns:
            (vx, vy) 预测速度 (px/s)
        """
        if not self.fitted or self.control_points is None:
            return (0.0, 0.0)

        n = self.control_points.shape[1]
        t_max = (n - self.order) * self.knot_span
        t = min(t_future, t_max * 0.95)
        return self._evaluate_velocity(t)

    def predict_bbox(self, t_future: float, bbox_size: Tuple[float, float] = (50, 50)) -> Tuple[int, int, int, int]:
        """
        预测未来的边界框位置。

        Args:
            t_future: 未来时间偏移 (s)
            bbox_size: 边界框大小 (w, h)

        Returns:
            (x1, y1, x2, y2) 预测边界框
        """
        cx, cy = self.predict(t_future)
        w, h = bbox_size
        x1 = int(cx - w / 2.0)
        y1 = int(cy - h / 2.0)
        x2 = int(cx + w / 2.0)
        y2 = int(cy + h / 2.0)
        return (x1, y1, x2, y2)

    def get_prediction_trajectory(self, num_steps: int = 10) -> List[Tuple[float, float]]:
        """
        获取预测轨迹的采样点列表。

        Args:
            num_steps: 采样点数

        Returns:
            [(x, y), ...] 轨迹采样点
        """
        if not self.fitted or self.control_points is None:
            return []

        n = self.control_points.shape[1]
        t_max = (n - self.order) * self.knot_span
        points = []
        for i in range(num_steps):
            t = t_max * i / (num_steps - 1) if num_steps > 1 else 0.0
            x, y = self._evaluate_bspline(t)
            points.append((x, y))
        return points

    def reset(self):
        """重置预测器状态。"""
        self.position_history.clear()
        self.control_points = None
        self.fitted = False
        self._basis_cache.clear()
