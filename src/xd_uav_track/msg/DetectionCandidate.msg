# 单个候选检测，用于 PixEagle 风格的 ID/空间/距离关联。

int32 track_id
int32 class_id
bool track_id_is_stable

int32[4] bbox
bool has_bbox

float32[4] normalized_bbox        # [cx, cy, w, h]
bool has_normalized_bbox

float32 confidence

# Optional L2-normalized visual embedding for C++ appearance ReID.
# Empty keeps all existing detector publishers fully compatible.
float32[] appearance_embedding

# Optional metric relative state in body frame (forward, right, down).
# The tracker/follower keep the legacy bbox path when these fields are absent.
bool has_relative_position_body
float32[3] relative_position_body
bool has_relative_velocity_body
float32[3] relative_velocity_body
float32[9] position_covariance
float32[9] velocity_covariance
bool range_valid

