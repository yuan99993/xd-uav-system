# FollowerCommand.msg — 跟随器控制指令消息
# 由 Follower 输出，供 PX4 Offboard 控制器或下游 Controller 使用。
#
# 对应 PX4 MAVSDK: offboard.set_velocity_body()
# 或 MAVROS: mavros/setpoint_raw/attitude (for yaw rate)

std_msgs/Header header

# ── 控制模式 ─────────────────────────────────────────────────────────────
string control_mode                # "velocity_body" | "attitude_rate"
string lateral_guidance_mode       # 当前侧向制导模式
string follower_profile            # PixEagle-compatible profile name
bool profile_supported             # 当前输出链是否支持该 profile
string profile_rejection_reason    # fail-closed reason when unsupported

# ── Body 速度指令 (m/s, body frame: x=forward, y=right, z=down) ──────────
float32 velocity_forward           # 前向速度 (m/s)
float32 velocity_right             # 右向速度 (m/s) — sideslip 模式使用
float32 velocity_down              # 下降速度 (m/s) — 正值=向下

# ── 偏航速率指令 (deg/s) ─────────────────────────────────────────────────
float32 yaw_rate_deg_s             # 偏航角速率 (deg/s)
float32 yaw_rate_raw_deg_s         # 原始 PID 输出（平滑前）
bool yaw_smoothing_active          # 偏航平滑是否激活

# ── 姿态率指令（仅 attitude_rate profile）───────────────────────────────
float32 roll_rate_deg_s
float32 pitch_rate_deg_s
float32 thrust                     # normalized [0, 1]

# ── 指令有效性 ───────────────────────────────────────────────────────────
bool command_valid                 # 指令是否有效
bool target_visible                # 目标是否可见
bool emergency_stop_active         # 紧急停止状态

# ── 自适应信息 ───────────────────────────────────────────────────────────
bool adaptive_dive_climb_active    # 自适应俯冲/爬升是否激活
float32 adaptive_correction_down   # 自适应垂直修正 (m/s)
float32 adaptive_correction_fwd    # 自适应前向修正 (m/s)

# ── 俯仰补偿 ─────────────────────────────────────────────────────────────
bool pitch_compensation_active     # 俯仰补偿是否激活
float32 pitch_compensation_value   # 俯仰补偿值 (归一化坐标)

# ── 目标信息 ─────────────────────────────────────────────────────────────
float32 target_error_x             # 当前 X 误差（归一化）
float32 target_error_y             # 当前 Y 误差（归一化）
bool target_lost                   # 目标是否丢失
float32 target_loss_duration       # 目标丢失持续时间 (s)

# ── PID 信息 ─────────────────────────────────────────────────────────────
float32 pid_yaw_output             # 偏航 PID 输出 (rad/s)
float32 pid_down_output            # 垂直 PID 输出 (m/s)
float32 pid_right_output           # 侧向 PID 输出 (m/s)

# ── 不确定性/三维状态诊断 ───────────────────────────────────────────────
bool metric_state_active
float32 uncertainty_scale
bool uncertainty_limited
bool uncertainty_abort

# ── 产品级控制守卫 ───────────────────────────────────────────────────────
uint64 command_sequence
time source_capture_timestamp
string control_state               # DISABLED / FOLLOWING / TARGET_LOST / ...
string invalid_reason              # command_valid=false 的可审计原因
string output_backend              # command_only / mavros_body / mrs_velocity
bool control_authorized            # 控制租约或兼容授权是否有效
bool platform_ready                # 所选执行后端状态是否允许输出
