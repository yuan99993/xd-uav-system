import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from sar_yolo_detector.box_fusion import BoxFusion, Candidate, class_duplicate_indices, iou


class LocalTracker:
    tracker_name = "fake_kcf"

    def __init__(self):
        self.box = None
        self.valid = True
        self.updates = 0
        self.initializations = 0

    def reset(self):
        self.box = None

    def initialize(self, frame, box):
        self.box = tuple(box)
        self.initializations += 1

    def update(self, frame):
        self.updates += 1
        return SimpleNamespace(bbox=self.box, valid=self.valid)


def image(box=(60, 60, 40, 40), background=24):
    frame = np.full((200, 260, 3), background, np.uint8)
    x, y, w, h = (int(v) for v in box)
    rng = np.random.RandomState(3)
    texture = rng.randint(0, 120, (h, w, 1)).astype(np.uint8)
    frame[y:y+h, x:x+w] = texture + np.array([20, 70, 10], np.uint8)
    return frame


class ClassDuplicateTest(unittest.TestCase):
    def candidate(self, box=(60, 60, 40, 40), category=2, confidence=.9, identity=5):
        return Candidate(box, category, confidence, identity)

    def test_conflicting_vehicle_classes_keep_stronger_box(self):
        candidates = [self.candidate(confidence=.8), self.candidate(category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [1])

    def test_selected_id_wins_without_changing_class_or_box(self):
        candidates = [self.candidate(confidence=.6), self.candidate(category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]], 5), [0])

    def test_nearby_vehicles_are_not_duplicates(self):
        candidates = [self.candidate(), self.candidate((75, 60, 40, 40), category=3, identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0, 1])

    def test_same_class_competitors_remain_for_ambiguity_gate(self):
        candidates = [self.candidate(), self.candidate(identity=7)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0, 1])

    def test_different_parents_and_nested_background_boxes_remain(self):
        candidates = [self.candidate(), self.candidate(category=4),
                      self.candidate((20, 20, 120, 120), category=3)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3], [4]]), [0, 1, 2])

    def test_empty_input_and_stable_tie_break(self):
        self.assertEqual(class_duplicate_indices([], [[0, 1, 2, 3]]), [])
        candidates = [self.candidate(), self.candidate(category=3)]
        self.assertEqual(class_duplicate_indices(candidates, [[0, 1, 2, 3]]), [0])


class BoxFusionTest(unittest.TestCase):
    def setUp(self):
        self.local = LocalTracker()
        self.fusion = BoxFusion(tracker=self.local)
        self.frame = image()
        self.candidate = Candidate((60, 60, 40, 40), 2, .90, 5)
        self.fusion.initialize(self.frame, self.candidate.box, 10., 900000001, self.candidate)

    def test_agreement_returns_one_fused_box(self):
        result = self.fusion.step(self.frame, 10.1, [self.candidate])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.public_id, 900000001)
        self.assertTrue(result.observed)
        self.assertEqual(self.local.initializations, 1)

    def test_strong_detector_not_pulled_back_by_valid_local_lag(self):
        candidate = Candidate((62, 60, 40, 40), 2, .9, 5)
        result = self.fusion.step(image(candidate.box), 10.1, [candidate])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.box, candidate.box)

    def test_moderate_detector_still_blends_compatible_witnesses(self):
        candidate = Candidate((62, 60, 40, 40), 2, .7, 5)
        result = self.fusion.step(image(candidate.box), 10.1, [candidate])
        self.assertEqual(result.source, "fused")
        self.assertGreater(result.box[0], 60.)
        self.assertLess(result.box[0], 62.)

    def test_missing_yolo_uses_visual_not_detector(self):
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertEqual(result.source, "visual")
        self.assertEqual(result.last_detection_stamp, 10.)
        self.assertLessEqual(result.confidence, .65)

    def test_weak_detection_maintains_only_agreeing_current_identity(self):
        weak = Candidate(self.candidate.box, 2, .2, 5)
        result = self.fusion.step(self.frame, 10.1, [weak])
        self.assertEqual(result.source, "fused")
        self.assertEqual(result.last_detection_stamp, 10.1)
        self.assertEqual(result.public_id, 900000001)

    def test_weak_detection_cannot_recover_new_local_identity(self):
        self.local.valid = False
        weak = Candidate(self.candidate.box, 2, .2, 17)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [weak])
        self.assertFalse(result.observed)
        self.assertEqual(result.local_id, 5)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_unknown_manual_visual_hold_does_not_refresh_itself(self):
        self.fusion.initialize(self.frame, self.candidate.box, 10., 900000002)
        for stamp in np.arange(10.1, 12., .1):
            result = self.fusion.step(self.frame, float(stamp), [])
        self.assertFalse(result.observed)
        self.assertEqual(result.last_detection_stamp, 0.)

    def test_visual_expiry_not_indefinite(self):
        for stamp in np.arange(10.1, 12., .1):
            result = self.fusion.step(self.frame, float(stamp), [])
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_failed_tracker_uses_yolo_and_reinitializes(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [self.candidate])
        self.assertEqual(result.source, "yolo")
        self.assertEqual(self.local.initializations, 2)

    def test_one_failed_capture_does_not_require_same_id_rebinding(self):
        self.local.valid = False
        self.assertFalse(self.fusion.step(self.frame, 10.1, [], detector_updated=False).observed)
        result = self.fusion.step(self.frame, 10.1,
            [Candidate(self.candidate.box, 2, .9, 5, True, 10.1)], same_frame_correction=True)
        self.assertTrue(result.observed, result.reason)
        self.assertEqual(result.local_id, 5)
        self.assertEqual(self.fusion.pending_count, 0)

    def test_valid_same_id_flow_maintains_lock_when_kcf_fails(self):
        self.local.valid = False
        cue = Candidate((62, 60, 40, 40), 2, .65, 5, True, 10.)
        result = self.fusion.step(image(cue.box), 10.1, [], detector_updated=False,
            visual_observation=cue, visual_quality=.9)
        self.assertEqual(result.source, 'visual')
        self.assertEqual(result.box, cue.box)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_flow_cannot_rebind_or_bypass_quality_expiry_or_bounds(self):
        self.local.valid = False
        for cue, quality in ((Candidate(self.candidate.box, 2, .65, 17, True, 10.), .9),
                             (Candidate(self.candidate.box, 2, .65, 5, True, 10.), .3),
                             (Candidate((170, 60, 40, 40), 2, .65, 5, True, 10.), .9),
                             (Candidate(self.candidate.box, 2, .65, 5, True, 8.), .9)):
            fusion = BoxFusion(tracker=LocalTracker())
            fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
            fusion.tracker.valid = False
            result = fusion.step(self.frame, 10.1, [], detector_updated=False,
                visual_observation=cue, visual_quality=quality)
            self.assertFalse(result.observed)
            self.assertEqual(fusion.local_id, 5)

    def test_detector_refreshes_kcf_at_bounded_cadence(self):
        for stamp in (10.2, 10.4, 10.6):
            self.fusion.step(self.frame, stamp, [self.candidate])
        self.assertEqual(self.local.initializations, 1)
        self.fusion.step(self.frame, 10.8, [self.candidate])
        self.assertEqual(self.local.initializations, 2)

    def test_flow_scale_metadata_cannot_bypass_scale_gate(self):
        cue = Candidate(self.candidate.box, 2, .65, 5, True, 10.)
        for ratio in (0., .1, 3., float('nan'), float('inf')):
            fusion = BoxFusion(tracker=LocalTracker())
            fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
            fusion.tracker.valid = False
            result = fusion.step(self.frame, 10.1, [], detector_updated=False,
                visual_observation=cue, visual_quality=.9, visual_scale_ratio=ratio)
            self.assertFalse(result.observed)

    def test_prediction_expires_and_is_not_observed(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertEqual(result.source, "prediction")
        self.assertFalse(result.observed)
        result = self.fusion.step(self.frame, 10.5, [])
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_competing_identical_vehicles_fail_closed(self):
        competitor = Candidate((61, 60, 40, 40), 2, .9, 6)
        result = self.fusion.step(self.frame, 10.1, [self.candidate, competitor])
        self.assertEqual(result.source, "ambiguous")
        self.assertFalse(result.observed)

    def test_new_local_id_needs_consecutive_confirmation(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2):
            result = self.fusion.step(self.frame, stamp, [new])
            self.assertEqual(result.source, "visual")
            self.assertTrue(result.observed)
            self.assertEqual(result.local_id, 5)
            self.assertEqual(result.last_detection_stamp, 10.)
        result = self.fusion.step(self.frame, 10.3, [new])
        self.assertTrue(result.observed)
        self.assertEqual(result.local_id, 17)
        self.assertEqual(result.public_id, 900000001)

    def test_confirmation_without_valid_visual_still_fails_closed(self):
        self.local.valid = False
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2):
            result = self.fusion.step(self.frame, stamp, [new])
            self.assertEqual(result.source, "ambiguous")
            self.assertFalse(result.observed)
        self.assertEqual(self.fusion.step(self.frame, 10.3, [new]).local_id, 17)

    def test_confirmation_does_not_extend_visual_hold_budget(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.5, 11., 11.5):
            self.fusion.step(self.frame, stamp, [])
        result = self.fusion.step(self.frame, 11.6, [new])
        self.assertFalse(result.observed)
        self.assertEqual(result.last_detection_stamp, 10.)

    def test_confirmation_visual_must_agree_with_candidate(self):
        # A motion-compatible box can still contradict the local observation.
        other = Candidate((85, 60, 40, 40), 2, .95, 17)
        frame = image()
        frame[60:100, 85:125] = self.frame[60:100, 60:100]
        result = self.fusion.step(frame, 10.1, [other])
        self.assertFalse(result.observed)

    def test_two_simultaneously_visible_tracks_never_stitched(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        far_old = Candidate((180, 120, 40, 40), 2, .9, 5)
        result = self.fusion.step(self.frame, 10.1, [new, far_old])
        self.assertEqual(result.reason, "old_and_new_tracks_simultaneously_visible")
        self.assertFalse(result.observed)

    def test_unstable_local_identity_not_rebound(self):
        self.local.valid = False
        new = Candidate(self.candidate.box, 2, .95, 17, False)
        self.assertFalse(self.fusion.step(self.frame, 10.1, [new]).observed)

    def test_duplicate_timestamp_does_not_confirm(self):
        new = Candidate(self.candidate.box, 2, .95, 17)
        self.fusion.step(self.frame, 10.1, [new])
        updates = self.local.updates
        for _ in range(4):
            self.assertFalse(self.fusion.step(self.frame, 10.1, [new]).observed)
        self.assertEqual(self.local.updates, updates)
        self.assertEqual(self.fusion.pending_count, 1)

    def test_switching_pending_candidate_resets_confirmation(self):
        self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 2, .95, 17)])
        self.fusion.step(self.frame, 10.2, [Candidate(self.candidate.box, 2, .95, 18)])
        self.assertEqual(self.fusion.pending_count, 1)

    def test_wrong_class_rejected(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 8, .95, 5)])
        self.assertEqual(result.source, "prediction")

    def test_vehicle_class_flip_does_not_change_identity(self):
        result = self.fusion.step(self.frame, 10.1, [Candidate(self.candidate.box, 1, .95, 5)])
        self.assertTrue(result.observed)
        self.assertEqual(result.class_id, 2)
        self.assertEqual(result.public_id, 900000001)

    def test_manual_unknown_not_fabricated_as_tank(self):
        result = self.fusion.initialize(self.frame, self.candidate.box, 10., 900000002)
        self.assertEqual(result.class_id, -1)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [self.candidate])
        self.assertEqual(result.class_id, 2)
        self.assertEqual(result.public_id, 900000002)

    def test_low_quality_background_not_visual_lock(self):
        result = self.fusion.initialize(np.full_like(self.frame, 24), self.candidate.box, 10., 900000002)
        self.assertEqual(result.source, "lost")
        self.assertIsNone(result.box)

    def test_false_same_id_wrong_position_rejected(self):
        false = Candidate((170, 60, 40, 40), 2, .99, 5)
        self.local.valid = False
        self.assertFalse(self.fusion.step(image(false.box), 10.1, [false]).observed)

    def test_nan_candidate_ignored(self):
        false = Candidate((float("nan"), 60, 40, 40), 2, .99, 5)
        self.local.valid = False
        self.assertFalse(self.fusion.step(self.frame, 10.1, [false]).observed)

    def test_cumulative_scale_drift_rejected(self):
        self.local.box = (30, 30, 100, 100)
        result = self.fusion.step(self.frame, 10.1, [])
        self.assertFalse(result.observed)

    def test_long_gap_disables_local_tracker(self):
        result = self.fusion.step(self.frame, 11., [])
        self.assertFalse(result.observed)
        self.assertIsNone(self.local.box)

    def test_frame_dimensions_change_fails_closed(self):
        result = self.fusion.step(np.zeros((100, 100, 3), np.uint8), 10.1, [])
        self.assertEqual(result.reason, "image_dimensions_changed")

    def test_long_recovery_rejects_unknown_camera_motion(self):
        self.local.valid = False
        result = self.fusion.step(self.frame, 19., [self.candidate])
        self.assertEqual(result.reason, "identity_memory_waiting")
        self.assertFalse(result.observed)
        self.assertIsNotNone(self.fusion.identity_archive)

    def _long_absence(self):
        self.local.valid = False
        self.fusion.velocity[:] = 0.
        for stamp in np.arange(10.2, 19., .2):
            self.fusion.step(self.frame, float(stamp), [], camera_transform=np.eye(3))

    def test_long_recovery_rebinds_only_after_three_fresh_detections(self):
        self._long_absence()
        new = Candidate(self.candidate.box, 2, .95, 17)
        initializations = self.local.initializations
        for stamp in (19., 19.2):
            result = self.fusion.step(self.frame, stamp, [new], camera_transform=np.eye(3))
            self.assertEqual(result.source, "ambiguous")
            self.assertEqual(result.local_id, 5)
        result = self.fusion.step(self.frame, 19.4, [new], camera_transform=np.eye(3))
        self.assertTrue(result.observed)
        self.assertEqual(result.public_id, 900000001)
        self.assertEqual(result.local_id, 17)
        self.assertEqual(self.local.initializations, initializations + 1)
        self.assertIsNone(self.fusion.identity_archive)
        np.testing.assert_equal(self.fusion.velocity, [0., 0.])

    def test_long_recovery_same_local_id_also_requires_confirmation(self):
        self._long_absence()
        result = self.fusion.step(self.frame, 19., [self.candidate], camera_transform=np.eye(3))
        self.assertEqual(result.source, "ambiguous")

    def test_long_recovery_never_accepts_appearance_without_enough_motion_evidence(self):
        self._long_absence()
        far = Candidate((110, 60, 40, 40), 2, .99, 17)
        for stamp in (19., 19.2, 19.4):
            result = self.fusion.step(image(far.box), stamp, [far], camera_transform=np.eye(3))
        self.assertFalse(result.observed)
        self.assertIn('long_identity_score', self.fusion.diagnostics['rejected'])

    def test_rotation_only_or_untrusted_motion_is_not_long_identity_evidence(self):
        self.local.valid = False
        for stamp in np.arange(10.2, 19.6, .2):
            result = self.fusion.step(self.frame, float(stamp), [self.candidate] if stamp >= 19 else [],
                camera_transform=np.eye(3), camera_motion_reliable=False)
        self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 0)
        self.assertIn('camera_motion_chain_untrusted', self.fusion.diagnostics['rejected'])

    def test_long_recovery_competitors_do_not_restore_original_id(self):
        self._long_absence()
        candidates = [Candidate(self.candidate.box, 2, .95, identity) for identity in (17, 18)]
        for stamp in (19., 19.2, 19.4):
            result = self.fusion.step(self.frame, stamp, candidates, camera_transform=np.eye(3))
            self.assertEqual(result.reason, "competing_detections")
            self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 0)

    def test_identity_archive_ttl_is_not_refreshed_by_pending_matches(self):
        self._long_absence()
        expiry = self.fusion.identity_archive['expires_at']
        self.fusion.step(self.frame, 19., [Candidate(self.candidate.box, 2, .95, 17)], camera_transform=np.eye(3))
        self.assertEqual(self.fusion.identity_archive['expires_at'], expiry)
        result = self.fusion.step(self.frame, 70.1, [self.candidate])
        self.assertEqual(result.reason, "identity_memory_expired")
        self.assertFalse(result.observed)
        self.assertIsNone(self.fusion.identity_archive)
        self.assertIsNone(self.fusion.anchor)

    def test_rebound_clears_stale_velocity_even_for_short_absence(self):
        self.local.valid = False
        self.fusion.velocity[:] = (1., 0.)
        new = Candidate(self.candidate.box, 2, .95, 17)
        for stamp in (10.1, 10.2, 10.3):
            result = self.fusion.step(self.frame, stamp, [new])
        self.assertTrue(result.observed)
        np.testing.assert_equal(self.fusion.velocity, [0., 0.])

    def test_identity_memory_configuration_bounds(self):
        for ttl in (8., 301., float('inf')):
            with self.assertRaises(ValueError):
                BoxFusion({'identity_memory_ttl_sec': ttl})

    def test_pending_same_id_spatial_jump_cannot_confirm(self):
        self.local.valid = False
        for stamp, x in ((10.1, 60), (10.2, 90), (10.3, 60)):
            candidate = Candidate((x, 60, 40, 40), 2, .95, 17)
            result = self.fusion.step(image(candidate.box), stamp, [candidate])
        self.assertFalse(result.observed)
        self.assertEqual(self.fusion.pending_count, 1)
        self.assertTrue(self.fusion.diagnostics['pending_motion_rejected'])

    def test_actual_kcf_long_absence_reinitializes_and_recovers_public_id(self):
        if not hasattr(cv2, 'TrackerKCF_create') and not hasattr(getattr(cv2, 'legacy', object()), 'TrackerKCF_create'):
            self.skipTest('OpenCV contrib KCF not installed')
        fusion = BoxFusion(manual_config={'feature_tracker': {'tracker_algorithm': 'kcf'}})
        fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
        blank = np.full_like(self.frame, 24)
        for index in range(1, 91):
            result = fusion.step(blank, 10.+index*.1, [], camera_transform=np.eye(3))
        self.assertFalse(result.observed)
        new = Candidate((70, 60, 40, 40), 2, .95, 17)
        for stamp in (19.1, 19.2, 19.3):
            result = fusion.step(image(new.box), stamp, [new], camera_transform=np.eye(3))
        self.assertTrue(result.observed)
        self.assertEqual(result.public_id, 99)
        self.assertEqual(result.local_id, 17)
        self.assertTrue(fusion.diagnostics['local_reinitialized'])
        self.assertGreater(iou(result.box, new.box), .99)

    def test_motion_uses_elapsed_time(self):
        second = Candidate((64, 60, 40, 40), 2, .9, 5)
        self.local.valid = False
        self.fusion.step(image(second.box), 10.2, [second])
        self.assertAlmostEqual(self.fusion.velocity[0], 8.)
        result = self.fusion.step(image(second.box), 10.3, [])
        self.assertEqual(result.source, "prediction")
        self.assertAlmostEqual(result.box[0], 64.8)

    def test_invalid_config_rejected(self):
        for config in ({"visual_hold_sec": -1}, {"detector_weight": 2}, {"max_frame_gap_sec": float("nan")}):
            with self.assertRaises(ValueError):
                BoxFusion(config)

    def test_actual_kcf_tracks_textured_moving_target(self):
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("OpenCV contrib KCF not installed")
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        fusion.initialize(self.frame, self.candidate.box, 10., 99, self.candidate)
        self.assertEqual(fusion.tracker.tracker_name, "kcf")
        for index in range(1, 11):
            box = (60 + index, 60, 40, 40)
            detection = Candidate(box, 2, .9, 5)
            result = fusion.step(image(box), 10 + index * .05, [] if index % 3 else [detection])
            self.assertTrue(result.observed, result.reason)
            self.assertGreater(iou(result.box, box), .70)
        self.assertEqual(result.public_id, 99)

    def test_actual_kcf_turn_gap_and_new_id_keep_all_120_observations(self):
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("OpenCV contrib KCF not installed")
        texture = np.random.RandomState(31).randint(0, 200, (40, 40, 3), dtype=np.uint8)
        def scene(index):
            frame = np.full((200, 320, 3), 20, np.uint8)
            frame[70:110, 60+index:100+index] = cv2.warpAffine(texture,
                cv2.getRotationMatrix2D((20, 20), index*.7, 1.), (40, 40), borderMode=cv2.BORDER_REFLECT)
            return frame
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        fusion.initialize(scene(0), (60, 70, 40, 40), 10., 99, Candidate((60, 70, 40, 40), 2, .9, 5))
        for index in range(1, 121):
            box, fresh = (60+index, 70, 40, 40), index % 5 == 0
            detections = [Candidate(box, 2, .9, 5 if index < 50 else 17)] if fresh and not 60 <= index <= 75 else []
            result = fusion.step(scene(index), 10.+index/30., detections, detector_updated=fresh)
            self.assertTrue(result.observed, "%d: %s" % (index, result.reason))
            self.assertEqual(result.public_id, 99)
            self.assertGreater(iou(result.box, box), .85)
            if detections and fusion.local_id == detections[0].local_id:
                self.assertEqual(result.box, box)


if __name__ == "__main__":
    unittest.main()
