# FollowerStatus.msg — 跟随器状态消息
# 包含跟随器的完整运行状态和诊断信息。

std_msgs/Header header

# ── 基本状态 ─────────────────────────────────────────────────────────────
bool following_active              # 跟随是否激活
bool target_lost                   # 目标是否丢失
bool emergency_stop_active         # 紧急停止
string control_mode                # 当前控制模式
string lateral_guidance_mode       # 侧向制导模式

# ── 速度状态 ─────────────────────────────────────────────────────────────
float32 current_forward_velocity   # 当前前向速度 (m/s)
float32 target_forward_velocity    # 目标前向速度 (m/s)
float32 lateral_velocity           # 侧向速度 (m/s)
float32 vertical_velocity          # 垂直速度 (m/s)
float32 yaw_rate                   # 偏航速率 (deg/s)

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
uint32 update_count                # 更新计数
