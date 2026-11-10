#!/usr/bin/env python3
"""No stale box can masquerade as an observed or predicted lock."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Header
from xd_uav_track.msg import PerceptionTargetArray
from xd_uav_track.sot_tracker import SotResult
from test_perception_modes import manager_fixture, frame, target, Publisher

path = Path(__file__).resolve().parents[1] / "scripts/perception_overlay_node.py"
spec = importlib.util.spec_from_file_location("canonical_overlay_test", str(path))
overlay_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(overlay_module)


class LossDisplayTest(unittest.TestCase):
    def setUp(self):
        clock = patch.object(rospy.Time, "now", return_value=rospy.Time(10))
        clock.start()
        self.addCleanup(clock.stop)

    def test_sot_failure_is_identity_only_not_prediction_or_old_coordinates(self):
        manager = manager_fixture()
        manager._mode = "track"  # SOT-only frames are the track-mode contract.
        stamp = rospy.Time(10)
        manager._frames["fixed"].append((1, stamp, frame()))
        manager._latest_headers["fixed"] = Header(stamp=stamp)
        manager._select_existing("fixed", target(), "template")
        manager._manual_tracker = SimpleNamespace(update=lambda *_: SotResult(
            None, False, 0.0, "template", "tracker_update_failed"))
        failed = rospy.Time.from_sec(10.1)
        manager._update_sot(frame(), failed)
        manager._publish_targets("fixed", Header(stamp=failed), 180, 120)
        lost = manager._targets_pub.messages[-1].targets[0]
        self.assertEqual(7, lost.track_id)
        self.assertEqual(("lost", "stale"), (lost.lifecycle_state, lost.measurement_source))
        self.assertEqual([0, 0, 0, 0], list(lost.bbox_xyxy))
        self.assertFalse(lost.position_valid)
        self.assertEqual([50, 40, 86, 64], list(manager._sot_at(failed).bbox_xyxy),
                         "internal recovery evidence must not be destroyed by presentation")

    def test_real_prediction_expires_and_unproven_prediction_is_hidden(self):
        manager = manager_fixture(); manager._mode = "detect"
        observed = target()
        manager._latest_targets["fixed"] = [observed]
        manager._latest_headers["fixed"] = Header(stamp=rospy.Time(10))
        manager._publish_targets("fixed", Header(stamp=rospy.Time(10)), 180, 120)
        predicted = deepcopy(observed)
        predicted.measurement_source = "prediction"; predicted.lifecycle_state = "occluded"
        for seconds, expected in ((10.2, "prediction"), (10.6, "stale")):
            stamp = rospy.Time.from_sec(seconds)
            manager._latest_targets["fixed"] = [predicted]
            manager._latest_headers["fixed"] = Header(stamp=stamp)
            with patch.object(rospy.Time, "now", return_value=stamp):
                manager._publish_targets("fixed", Header(stamp=stamp), 180, 120)
            output = manager._targets_pub.messages[-1].targets[0]
            self.assertEqual(expected, output.measurement_source)
            self.assertFalse(output.position_valid)
        manager._image_evidence.clear()
        manager._publish_targets("fixed", Header(stamp=stamp), 180, 120)
        self.assertEqual("stale", manager._targets_pub.messages[-1].targets[0].measurement_source)

    def test_camera_stop_publishes_cleared_snapshot_once(self):
        manager = manager_fixture(); manager._mode = "detect"
        stamp = rospy.Time(10)
        manager._frames["fixed"].append((1, stamp, frame()))
        manager._latest_targets["fixed"] = [target()]
        manager._latest_headers["fixed"] = Header(stamp=stamp)
        manager._last_source_wall["fixed"] = time.monotonic() - 2
        manager._heartbeat(None)
        output = manager._targets_pub.messages[-1].targets[0]
        self.assertEqual("stale", output.measurement_source)
        self.assertEqual([0, 0, 0, 0], list(output.bbox_xyxy))
        manager._heartbeat(None)
        self.assertEqual(1, len(manager._targets_pub.messages))

    def render(self, item):
        overlay = overlay_module.PerceptionOverlay.__new__(overlay_module.PerceptionOverlay)
        overlay._source = "fixed"; overlay._bridge = CvBridge()
        overlay._lock = threading.Lock(); overlay._publisher = Publisher()
        image = overlay._bridge.cv2_to_imgmsg(np.zeros((140, 180, 3), np.uint8), "bgr8")
        targets = PerceptionTargetArray(image_source="fixed", image_width=180,
            image_height=140, mode="track", targets=[item])
        overlay._matching_image = lambda _: image
        overlay._pending = targets
        with patch.object(overlay_module.cv2, "rectangle", wraps=overlay_module.cv2.rectangle) as rect, \
             patch.object(overlay_module.cv2, "line", wraps=overlay_module.cv2.line) as line, \
             patch.object(overlay_module.cv2, "putText", wraps=overlay_module.cv2.putText) as text:
            overlay._try_publish(targets)
            return rect.call_args_list, line.call_args_list, [call.args[1] for call in text.call_args_list]

    def test_overlay_lost_has_status_only_no_box(self):
        item = target(); item.lifecycle_state = "lost"; item.measurement_source = "stale"
        rectangles, lines, labels = self.render(item)
        self.assertEqual(1, len(rectangles), "only the status panel may be rectangular")
        self.assertEqual([], lines)
        self.assertTrue(any("LOST ID 7" in text for text in labels))

    def test_header_panel_cannot_erase_selected_box_near_top_edge(self):
        overlay = overlay_module.PerceptionOverlay.__new__(overlay_module.PerceptionOverlay)
        overlay._source='fixed'; overlay._bridge=CvBridge(); overlay._lock=threading.Lock()
        overlay._publisher=Publisher()
        image=overlay._bridge.cv2_to_imgmsg(np.zeros((140,180,3),np.uint8),'bgr8')
        selected=target(); selected.selected=True
        selected.bbox_xyxy=[80,4,105,23]; selected.center_px=[92.5,13.5]
        snapshot=PerceptionTargetArray(image_source='fixed',image_width=180,image_height=140,
            mode='fusion',targets=[selected])
        overlay._matching_image=lambda _:image
        overlay._pending=snapshot
        overlay._try_publish(snapshot)
        rendered=overlay._bridge.imgmsg_to_cv2(overlay._publisher.messages[-1],'bgr8')
        crop=rendered[4:23,80:105]
        green=(crop[:,:,1]>180)&(crop[:,:,0]<100)&(crop[:,:,2]<100)
        self.assertGreater(int(green.sum()),30)

    def test_box_labels_only_show_class_and_id_but_prediction_stays_dashed(self):
        item = target(); item.lifecycle_state = "occluded"; item.measurement_source = "prediction"
        rectangles, lines, labels = self.render(item)
        self.assertEqual(1, len(rectangles)); self.assertGreater(len(lines), 0)
        self.assertIn("tank ID 7", labels)
        for source in ("detector", "visual_tracker"):
            item.lifecycle_state = "confirmed"; item.measurement_source = source
            _, _, labels = self.render(item)
            self.assertIn("tank ID 7", labels)
            self.assertFalse(any(tag in text for text in labels for tag in ("DET", "SOT", "PRED")))


if __name__ == "__main__":
    unittest.main()
