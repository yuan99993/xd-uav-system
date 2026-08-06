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

# ── SmartTracker 关联诊断/控制协同 ───────────────────────────────────────
# 关联来源：direct / id / spatial / distance / appearance / tentative / predicted / lost
string association_method
bool reidentification_match        # 本帧是否经外观 ReID 恢复
bool control_measurement_ready     # 此测量是否已通过重捕获确认，可供控制器使用
float32 control_confidence         # 已结合关联可靠性的控制置信度 [0.0, 1.0]

# ── 时间/滤波诊断 ───────────────────────────────────────────────────────
time capture_timestamp             # 检测/图像采集时间（不是发布时刻）
float32 inference_latency_ms       # 从采集时间到当前发布的估计延迟
float32[4] state_covariance        # [cx, cy, log_width, log_height] 对角协方差
string fusion_status               # FUSED_CURRENT/FUSED_OOSM/REJECT_NIS/PREDICTED/...
bool gmc_active                     # 本周期是否应用了视觉全局运动补偿
uint32 gmc_inlier_count             # GMC RANSAC 内点数

# ── 可选三维相对状态（body: forward, right, down） ──────────────────────
float32[3] target_relative_position_body
float32[3] target_relative_velocity_body
float32[9] position_covariance
float32[9] velocity_covariance
bool range_valid

# ── 元数据 ───────────────────────────────────────────────────────────────
string tracker_name               # 跟踪器名称
string tracker_type               # 跟踪器类型 (yolo/kalman/csrt/...)
