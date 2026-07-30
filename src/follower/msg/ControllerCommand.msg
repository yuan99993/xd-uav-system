# ControllerCommand.msg — 控制器指令消息
# Controller → Follower 方向:
# 下游控制器发送给 Follower 的执行反馈、修正或覆盖指令。

std_msgs/Header header

# ── 控制器标识 ───────────────────────────────────────────────────────────
string controller_name             # 控制器名称

# ── 指令状态 ─────────────────────────────────────────────────────────────
string command_status              # "accepted" | "modified" | "rejected" | "override" | "ready"

# ── 覆盖指令（当 command_status = "override" 时生效） ────────────────────
bool override_active               # 是否主动覆盖 Follower 输出
string override_reason             # 覆盖原因
float32 override_velocity_forward  # 覆盖前向速度 (m/s)
float32 override_velocity_right    # 覆盖右向速度 (m/s)
float32 override_velocity_down     # 覆盖下降速度 (m/s)
float32 override_yaw_rate_deg_s    # 覆盖偏航速率 (deg/s)

# ── 实际执行反馈 ─────────────────────────────────────────────────────────
float32 executed_velocity_forward  # 实际执行的前向速度 (m/s)
float32 executed_velocity_right    # 实际执行的右向速度 (m/s)
float32 executed_velocity_down     # 实际执行的下降速度 (m/s)
float32 executed_yaw_rate_deg_s    # 实际执行的偏航速率 (deg/s)

# ── 控制器状态 ───────────────────────────────────────────────────────────
bool controller_ready              # 控制器是否就绪
bool offboard_mode_active          # Offboard 模式是否激活
string flight_mode                 # 当前飞行模式

# ── 速度限制（控制器提供给 Follower 的约束） ──────────────────────────────
float32[6] velocity_limits         # [vx_min, vx_max, vy_min, vy_max, vz_min, vz_max] m/s
bool has_velocity_limits           # 是否提供了速度限制
