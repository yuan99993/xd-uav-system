# NormalizedError.msg — 归一化误差消息
# 供 Tracker → Follower 之间传递归一化跟踪误差。

std_msgs/Header header

# ── 归一化误差 ───────────────────────────────────────────────────────────
float32 error_x                   # 水平方向归一化误差 [-1, 1]
                                  # 负值 = 目标在左侧, 正值 = 目标在右侧
float32 error_y                   # 垂直方向归一化误差 [-1, 1]
                                  # 负值 = 目标在上方, 正值 = 目标在下方
float32 error_size                # 尺寸误差 [-1, 1]
                                  # 负值 = 目标太小（太远）, 正值 = 目标太大（太近）

# ── 误差有效性 ───────────────────────────────────────────────────────────
bool error_valid                  # 误差是否有效
bool target_visible               # 目标是否可见
bool is_estimated                 # 是否为估计值（非直接检测）

# ── 质量与生命周期 ───────────────────────────────────────────────────────
float32 confidence                # 当前检测/跟踪置信度 [0, 1]
float32 tracking_quality          # 综合质量（置信度、预测衰减、边缘惩罚）[0, 1]
uint32 frames_since_detection     # 自最后一帧直接检测后的帧数

# ── 时间戳 ───────────────────────────────────────────────────────────────
float32 dt_since_detection        # 自上次检测以来的时间（秒）

# ── SmartTracker → Follower 协同 ─────────────────────────────────────────
# Legacy publishers leave association_method empty.  Follower then falls back
# to error_valid/confidence semantics for backward compatibility.
string association_method          # direct / id / spatial / distance / appearance / tentative / predicted / lost
bool control_measurement_ready     # 重捕获已确认且允许驱动飞行控制
float32 control_confidence         # 置信度与关联来源质量融合后的控制置信度

# ── 时间/滤波诊断 ───────────────────────────────────────────────────────
time capture_timestamp
float32 inference_latency_ms
float32[4] state_covariance         # [cx, cy, log_width, log_height] 对角协方差
string fusion_status
bool gmc_active
uint32 gmc_inlier_count

# 可选三维相对状态，供 follower 优先使用；不可用时回退 bbox-size。
float32[3] target_relative_position_body
float32[3] target_relative_velocity_body
float32[9] position_covariance
float32[9] velocity_covariance
bool range_valid

# ── 相机/机体系视线角 ───────────────────────────────────────────────────
float32 yaw_error_rad             # target line-of-sight yaw relative to body
float32 pitch_error_rad           # target line-of-sight pitch relative to body
bool has_angular_error
