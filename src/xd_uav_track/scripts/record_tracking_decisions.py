#!/usr/bin/env python3
"""Optional bounded JSONL decision recorder. No flight commands are sent."""
import argparse
import json
import queue
import time

import rospy
from xd_uav_track.msg import TrackStatus, TrackStateArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/uav1")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("tracking_decisions", anonymous=True)
    records = queue.Queue(maxsize=1024)

    def emit(record):
        record["schema_version"] = 1
        record["evidence_scope"] = "tracker_decision_not_visual_ground_truth"
        record["receive_monotonic"] = time.monotonic()
        try:
            records.put_nowait(record)
        except queue.Full:
            rospy.logwarn_throttle(5.0, "decision recorder queue full: record dropped")

    def status(msg):
        emit(dict(kind="decision", stamp=msg.header.stamp.to_sec(),
                  target_id=msg.track_id, state=msg.tracking_state,
                  valid=msg.command_valid, predicted=msg.target_predicted,
                  reason=msg.invalid_reason, association=msg.association_method,
                  input_age=msg.input_age_sec,
                  profile=msg.follower_profile,
                  lateral_guidance=msg.lateral_guidance_mode,
                  command=[msg.command_forward, msg.command_left, msg.command_up,
                           msg.command_yaw_rate]))

    def tracks(msg):
        emit(dict(kind="observation", stamp=msg.header.stamp.to_sec(),
                  source=msg.image_source, frame=msg.header.frame_id,
                  selected=[dict(id=t.track_id, association=t.association_method,
                                 quality=t.tracking_quality, detected=t.detected,
                                 predicted=t.predicted) for t in msg.tracks if t.selected]))

    namespace = "/" + args.namespace.strip("/")
    rospy.Subscriber(namespace + "/track/status", TrackStatus, status, queue_size=10)
    rospy.Subscriber(namespace + "/track/tracks", TrackStateArray, tracks, queue_size=10)
    # Exclusive create prevents overwriting evidence from a previous run.
    with open(args.output, "x", encoding="utf-8") as stream:
        while not rospy.is_shutdown() or not records.empty():
            try:
                record = records.get(timeout=0.2)
            except queue.Empty:
                continue
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            stream.flush()


if __name__ == "__main__":
    main()
