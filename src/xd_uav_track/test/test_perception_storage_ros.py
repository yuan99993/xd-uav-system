#!/usr/bin/env python3
"""Real binary storage limits and idle TTL, without YOLO/Gazebo/flight nodes."""
import time
import unittest
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from xd_uav_track.msg import DetectionArray, DetectionCandidate, TrackStateArray


class StorageTest(unittest.TestCase):
    def setUp(self):
        if not rospy.core.is_initialized():
            self.skipTest("run with rostest perception_storage.test")

    def test_capacity_and_disconnected_source_expiry(self):
        root = "/perception_storage_test/"
        counts, frames = {}, []
        def diagnostic(message):
            for status in message.status:
                counts.update({value.key: int(value.value) for value in status.values})
        subscribers = [rospy.Subscriber(root + "track/runtime_diagnostics", DiagnosticArray, diagnostic),
                       rospy.Subscriber(root + "track/tracks_by_source", TrackStateArray, frames.append)]
        publisher = rospy.Publisher(root + "detect/detections", DetectionArray, queue_size=1)
        def wait(condition, seconds=4):
            deadline = time.monotonic() + seconds
            while not condition() and time.monotonic() < deadline: time.sleep(.02)
            self.assertTrue(condition(), "bounded wait expired")
        wait(lambda: publisher.get_num_connections() > 0 and "sources" in counts)
        def send(source, count=1):
            message = DetectionArray(image_source=source,image_width=640,image_height=480)
            message.header.stamp = rospy.Time.now()
            message.candidates = [DetectionCandidate(track_id=-1,class_id=2,confidence=.95,
                has_bbox=True,bbox=[10+(index%5)*120,10+(index//5)*100,
                                   50+(index%5)*120,50+(index//5)*100]) for index in range(count)]
            publisher.publish(message)
            return message.header.stamp
        stamp = send("alpha", 20)
        wait(lambda: any(frame.header.stamp == stamp for frame in frames))
        alpha = next(frame for frame in frames if frame.header.stamp == stamp)
        self.assertEqual(16, len(alpha.tracks))
        self.assertEqual(16, len({track.track_id for track in alpha.tracks}))
        # First observe the populated generation; the latched initial zero
        # snapshot is not evidence that this generation has expired.
        wait(lambda: counts.get("sources") == 1 and counts.get("global_entities") == 16)
        time.sleep(.05)
        send("beta"); send("unknown")
        time.sleep(.2)
        self.assertFalse(any(frame.image_source in ("beta","unknown") for frame in frames))
        # Timer must release all tables even when no subsequent images arrive.
        wait(lambda: counts.get("sources") == 0 and counts.get("global_entities") == 0)
        self.assertEqual(0, counts["local_mappings"])
        stamp = send("beta")
        wait(lambda: any(frame.header.stamp == stamp for frame in frames))
        beta = next(frame for frame in frames if frame.header.stamp == stamp)
        self.assertEqual("beta", beta.image_source)
        self.assertGreater(beta.tracks[0].track_id, max(track.track_id for track in alpha.tracks))
        for subscriber in subscribers: subscriber.unregister()
        publisher.unregister()


if __name__ == "__main__":
    rospy.init_node("perception_storage_test_runner")
    rostest.rosrun("xd_uav_track", "perception_storage", StorageTest)
