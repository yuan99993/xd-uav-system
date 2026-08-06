# 同一视频帧的候选检测集合。

std_msgs/Header header
tracker/DetectionCandidate[] candidates
string command                    # start_track / stop_track / reset / ""

# Product detector provenance. Empty values preserve legacy publishers.
string image_source               # eo / ir / logical stream name
string sensor_id
string detector_name
string model_version
