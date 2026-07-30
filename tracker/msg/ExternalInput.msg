# ExternalInput.msg — 外部输入消息
# 用于接收外部检测/特征识别系统的输入。

std_msgs/Header header

# ── 输入来源 ─────────────────────────────────────────────────────────────
string source                     # 输入来源: "bounding_box", "feature_point", "roi", "external_detector"

# ── 边界框输入 ───────────────────────────────────────────────────────────
int32[4] bbox                     # [x1, y1, x2, y2] 像素坐标
bool has_bbox

float32[4] normalized_bbox        # [cx, cy, w, h] 归一化坐标
bool has_normalized_bbox

# ── 特征点输入 ───────────────────────────────────────────────────────────
float32[2] feature_point          # [x, y] 归一化特征点坐标
bool has_feature_point

float32[2] feature_velocity       # [vx, vy] 归一化特征点速度
bool has_feature_velocity

# ── ROI 输入 ─────────────────────────────────────────────────────────────
float32[4] roi                    # [x, y, w, h] 归一化 ROI
bool has_roi

# ── 检测置信度 ───────────────────────────────────────────────────────────
float32 confidence                # 检测置信度 [0, 1]
int32 class_id                    # 可选类别 ID

# ── 控制命令 ─────────────────────────────────────────────────────────────
string command                    # "start_track", "stop_track", "reset", ""
