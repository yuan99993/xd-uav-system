# TrackingOutput.msg — 跟踪器统一输出消息
# Unified tracker output message, compatible with PixEagle TrackerOutput schema.

std_msgs/Header header

# ── 跟踪状态 ─────────────────────────────────────────────────────────────
bool tracking_active              # 是否正在跟踪
int32 target_id                   # 目标 ID（多目标场景）
int32 class_id                    # 目标类别 ID
float32 confidence                # 跟踪置信度 [0.0, 1.0]

# ── 位置数据（多种格式） ──────────────────────────────────────────────────
# 归一化 2D 位置 (相对于图像坐标系，[0, 1])
float32[2] position_2d            # [x, y] 归一化图像坐标
bool has_position_2d

# 3D 位置（带深度）
float32[3] position_3d            # [x, y, z] NED 或相机坐标系
bool has_position_3d

# ── 边界框数据 ───────────────────────────────────────────────────────────
# 像素坐标 (x1, y1, x2, y2)
int32[4] bbox                     # 像素边界框
bool has_bbox

# 归一化边界框
float32[4] normalized_bbox        # [cx, cy, w, h] 归一化
bool has_normalized_bbox

# 几何类型: "aabb", "obb", "polygon"
string geometry_type

# 旋转边界框 (cx, cy, w, h, rad)
float32[5] oriented_bbox
bool has_oriented_bbox

# ── 运动数据 ─────────────────────────────────────────────────────────────
float32[2] velocity               # [vx, vy] 像素/秒
bool has_velocity

float32[2] acceleration           # [ax, ay] 像素/秒^2
bool has_acceleration

# ── 归一化跟踪误差（供 Follower 使用） ────────────────────────────────────
float32 error_x                   # 归一化水平误差 [-1, 1]
float32 error_y                   # 归一化垂直误差 [-1, 1]
float32 error_size                # 归一化尺寸误差（目标大小 vs 期望大小）
bool has_normalized_error

# ── 质量指标 ─────────────────────────────────────────────────────────────
float32 tracking_quality          # 跟踪质量 [0.0, 1.0]
int32 frames_since_detection      # 自上次检测以来的帧数
bool is_predicted                 # 当前位置是否为预测值

# ── 元数据 ───────────────────────────────────────────────────────────────
string tracker_name               # 跟踪器名称
string tracker_type               # 跟踪器类型 (yolo/kalman/csrt/...)
