# Native-resolution annotated image and the exact target snapshot used to draw it.
# header.stamp/frame_id match the capture; rospy may replace header.seq with a
# transport sequence. Always select using targets.frame_seq/targets.header.stamp.
std_msgs/Header header
# Increases per source/overlay process, including updates of the same capture.
# Resets if that overlay process restarts. Use targets.session_id/model_epoch too.
uint64 frame_revision
xd_uav_track/PerceptionTargetArray targets
# Lossless PNG, format: "bgr8; png compressed bgr8". Never resized.
sensor_msgs/CompressedImage image
