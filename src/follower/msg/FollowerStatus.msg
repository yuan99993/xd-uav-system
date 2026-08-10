# FollowerStatus.msg — 跟随器状态消息
# 包含跟随器的完整运行状态和诊断信息。

std_msgs/Header header

# ── 基本状态 ─────────────────────────────────────────────────────────────
bool following_active              # 跟随是否激活
bool target_lost                   # 目标是否丢失
bool emergency_stop_active         # 紧急停止
string control_mode                # 当前控制模式
string lateral_guidance_mode       # 侧向制导模式
string follower_profile            # PixEagle-compatible profile name
string effective_profile           # 实际参与计算的 profile（可能是传感器降级 profile）
string previous_profile            # 无扰切换期间的来源 profile
bool profile_fallback_active       # GM 角度缺失时是否使用 MC 归一化误差降级
bool profile_transition_active     # 同控制域 profile 输出是否正在平滑交接
bool profile_supported
string profile_rejection_reason

# ── 速度状态 ─────────────────────────────────────────────────────────────
float32 current_forward_velocity   # 当前前向速度 (m/s)
float32 target_forward_velocity    # 目标前向速度 (m/s)
float32 lateral_velocity           # 侧向速度 (m/s)
float32 vertical_velocity          # 垂直速度 (m/s)
float32 yaw_rate                   # 偏航速率 (deg/s)
float32 roll_rate                  # 姿态率 profile 滚转速率 (deg/s)
float32 pitch_rate                 # 姿态率 profile 俯仰速率 (deg/s)
float32 thrust                     # 姿态率 profile 推力 [0, 1]

# ── 目标状态 ─────────────────────────────────────────────────────────────
float32 target_error_x             # X 误差
float32 target_error_y             # Y 误差
float32 target_confidence          # 目标置信度
float32 target_loss_duration       # 丢失持续时间 (s)

# ── 自适应状态 ───────────────────────────────────────────────────────────
bool adaptive_active               # 自适应模式是否激活
float32 vertical_rate_error        # 垂直速率误差 (px/s)
float32 smoothed_vertical_rate     # 平滑后垂直速率

# ── 俯仰补偿状态 ─────────────────────────────────────────────────────────
bool pitch_compensation_active     # 俯仰补偿激活
float32 current_pitch_angle        # 当前俯仰角 (deg)
float32 pitch_compensation_value   # 俯仰补偿值

# ── 安全状态 ─────────────────────────────────────────────────────────────
float32 altitude_current           # 当前高度 (m)
bool altitude_safe                 # 高度是否安全
int32 altitude_violation_count     # 高度违规次数

# ── 诊断 ─────────────────────────────────────────────────────────────────
float32 pid_yaw_output             # PID 偏航输出
float32 pid_down_output            # PID 垂直输出
float32 pid_right_output           # PID 侧向输出
float32 command_rate_hz            # 指令发布频率 (Hz)

# ── FPS / 性能 ───────────────────────────────────────────────────────────
float32 loop_actual_rate           # 实际循环频率 (Hz)
float32 input_age_ms               # 最新 tracker 消息年龄（兼容字段）
float32 compute_time_ms            # 本次 core 计算与约束耗时
float32 end_to_end_latency_ms      # tracker 采集到指令发布的估算总延迟
uint32 update_count                # 更新计数

# ── 端到端时延分解 ─────────────────────────────────────────────────────
float32 tracker_capture_age_ms       # 当前时刻 - 图像采集时间
float32 tracker_inference_latency_ms # tracker 报告的采集到发布延迟
float32 tracker_transport_latency_ms # 推理后传输/排队延迟
bool latency_limited                 # 时延超过告警阈值，已限幅
bool latency_abort                   # 时延超过控制安全阈值，拒绝目标控制

# ── 不确定性/三维状态诊断 ───────────────────────────────────────────────
float32 relative_position_sigma
float32 relative_velocity_sigma
float32 uncertainty_scale
bool metric_state_active
bool uncertainty_limited
bool uncertainty_abort

# ── 产品级控制守卫与状态机 ───────────────────────────────────────────────
string control_state
string invalid_reason
string output_backend
bool control_authorized
bool platform_ready
bool input_received
bool capture_timestamp_valid
string lease_requester
float32 lease_remaining_sec
uint64 rejected_input_count
uint64 rejected_output_count
