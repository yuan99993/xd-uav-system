#!/usr/bin/env python3
"""Write capture-stamped detections and tracker IDs for offline evaluation."""

import json
import os
import threading

import rospy
from std_msgs.msg import Header
from xd_uav_track.msg import DetectionArray, TrackStateArray


class Recorder:
    def __init__(self):
        path = os.path.abspath(os.path.expanduser(rospy.get_param("~output_path")))
        if not path.endswith(".jsonl"):
            raise ValueError("output_path must end in .jsonl")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.file = open(path, "w", buffering=1)
        self.lock = threading.Lock()
        self.counts = {"frame": 0, "detections": 0, "tracks": 0}
        self.source_a = str(rospy.get_param("~source_a", "replay_fixed"))
        self.source_b = str(rospy.get_param("~source_b", "replay_gimbal"))
        rospy.Subscriber("fixed_camera/frame_marker", Header,
                         lambda msg: self.frame(msg, self.source_a), queue_size=10)
        rospy.Subscriber("gimbal_camera/frame_marker", Header,
                         lambda msg: self.frame(msg, self.source_b), queue_size=10)
        rospy.Subscriber("track/detections", DetectionArray, self.detections,
                         queue_size=10)
        rospy.Subscriber("track/tracks_by_source", TrackStateArray, self.tracks,
                         queue_size=10)
        rospy.on_shutdown(self.close)
        rospy.loginfo("video tracking record: %s", path)

    def write(self, row):
        with self.lock:
            self.file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self.counts[row["type"]] += 1

    def detections(self, msg):
        self.write({"type": "detections", "t": msg.header.stamp.to_sec(),
                    "source": msg.image_source, "frame": msg.header.seq,
                    "objects": [{"class_id": c.class_id, "confidence": c.confidence,
                                 "bbox": list(c.bbox)} for c in msg.candidates]})

    def frame(self, header, source):
        self.write({"type": "frame", "t": header.stamp.to_sec(),
                    "source": source, "frame": int(header.frame_id)})

    def tracks(self, msg):
        self.write({"type": "tracks", "t": msg.header.stamp.to_sec(),
                    "source": msg.image_source, "frame": msg.header.seq,
                    "objects": [{"id": t.track_id, "class_id": t.class_id,
                                 "bbox": list(t.bbox), "state": t.lifecycle_state,
                                 "detected": t.detected, "selected": t.selected,
                                 "quality": t.tracking_quality}
                                for t in msg.tracks]})

    def close(self):
        with self.lock:
            if not self.file.closed:
                self.file.close()
        rospy.loginfo("video tracking record counts: %s", self.counts)


if __name__ == "__main__":
    rospy.init_node("video_tracking_record")
    Recorder()
    rospy.spin()
