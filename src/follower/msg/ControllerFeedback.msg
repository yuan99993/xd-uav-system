# ControllerFeedback.msg — Follower → Controller 方向
# Follower 向 Controller 发送的建议指令和状态信息。
#
# Controller 应订阅此话题获取 Follower 的实时建议。
# Controller 通过 ControllerCommand 话题返回执行反馈/修正。

std_msgs/Header header

# ── Follower 建议的控制指令（供 Controller 参考） ────────────────────────
float32 suggested_velocity_forward # 建议前向速度 (m/s)
float32 suggested_velocity_right   # 建议右向速度 (m/s)
float32 suggested_velocity_down    # 建议下降速度 (m/s)
float32 suggested_yaw_rate_deg_s   # 建议偏航速率 (deg/s)
bool suggestion_valid              # 建议是否有效

# ── 当前控制模式 ─────────────────────────────────────────────────────────
string control_mode                # "velocity_body" | "attitude_rate"
string lateral_guidance_mode       # "coordinated_turn" | "sideslip"
string follower_profile
bool profile_supported

# ── 目标状态 ─────────────────────────────────────────────────────────────
float32 target_error_x             # 当前 X 误差（归一化）
float32 target_error_y             # 当前 Y 误差（归一化）
bool target_visible                # 目标是否可见
bool emergency_stop_active         # 紧急停止状态

# ── Follower 状态 ────────────────────────────────────────────────────────
bool following_active              # 跟随是否激活
float32 follower_confidence        # 跟随器置信度 [0, 1]
