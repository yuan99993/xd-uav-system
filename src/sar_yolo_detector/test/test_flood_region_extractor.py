#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest

from sensor_msgs.msg import Image
from sar_yolo_detector.msg import FloodRegion
from sar_yolo_detector.srv import GetFloodRegions


class FloodRegionExtractorTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._messages = []
        self._publisher = rospy.Publisher('/sar_flood_test/mask', Image,
                                          queue_size=4)
        self._subscriber = rospy.Subscriber('/sar_flood_test/regions',
                                             type(self)._array_type(),
                                             self._callback, queue_size=10)

    @staticmethod
    def _array_type():
        from sar_yolo_detector.msg import FloodRegionArray
        return FloodRegionArray

    def _callback(self, message):
        with self._lock:
            self._messages.append(message)

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.rostime.wallsleep(0.01)
        return False

    @staticmethod
    def _mask(stamp, shift=0):
        width, height = 80, 50
        data = bytearray(width * height)
        for x0 in (8 + shift, 48 + shift):
            for y in range(12, 24):
                for x in range(x0, x0 + 12):
                    data[y * width + x] = 1
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = 'eo_optical'
        message.width = width
        message.height = height
        message.encoding = 'mono8'
        message.step = width
        message.data = bytes(data)
        return message

    def test_unique_tracks_duplicate_stamp_snapshot_and_expiry(self):
        self.assertTrue(self._wait(lambda:
            self._publisher.get_num_connections() > 0 and
            self._subscriber.get_num_connections() > 0))
        first_stamp = rospy.Time.now()
        self._publisher.publish(self._mask(first_stamp))
        self.assertTrue(self._wait(lambda: any(len(m.regions) == 2
                                               for m in self._messages)))
        # The exact same capture must not advance confirmation.
        with self._lock:
            before_duplicate = len(self._messages)
        self._publisher.publish(self._mask(first_stamp))
        rospy.rostime.wallsleep(0.10)
        with self._lock:
            self.assertEqual(before_duplicate, len(self._messages))

        second_stamp = rospy.Time.now()
        self._publisher.publish(self._mask(second_stamp, shift=1))
        self.assertTrue(self._wait(lambda: any(
            len(m.regions) == 2 and
            all(r.status == FloodRegion.CONFIRMED for r in m.regions)
            for m in self._messages)))
        with self._lock:
            confirmed = next(m for m in reversed(self._messages)
                             if len(m.regions) == 2 and
                             all(r.status == FloodRegion.CONFIRMED
                                 for r in m.regions))
        self.assertEqual(len({r.track_id for r in confirmed.regions}), 2)
        self.assertTrue(all(r.semantic_type == FloodRegion.FLOODED_BUILDING
                            for r in confirmed.regions))
        self.assertTrue(all(r.provenance.uav_id == 'uav_01'
                            for r in confirmed.regions))
        self.assertTrue(all(r.observation_count == 2
                            for r in confirmed.regions))

        rospy.wait_for_service('/sar_flood_test/regions/get_snapshot', 2.0)
        snapshot = rospy.ServiceProxy('/sar_flood_test/regions/get_snapshot',
                                      GetFloodRegions)().snapshot
        self.assertTrue(snapshot.full_snapshot)
        self.assertEqual(len(snapshot.regions), 2)

        self.assertTrue(self._wait(lambda: any(
            len(m.regions) == 2 and
            all(r.status == FloodRegion.EXPIRED for r in m.regions)
            for m in self._messages), timeout=2.0))


if __name__ == '__main__':
    rospy.init_node('sar_flood_region_extractor_test')
    rostest.rosrun('sar_yolo_detector', 'flood_region_extractor',
                   FloodRegionExtractorTest)
