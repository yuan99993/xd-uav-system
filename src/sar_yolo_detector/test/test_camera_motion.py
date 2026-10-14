import unittest
import cv2
import numpy as np

from sar_yolo_detector.camera_motion import CameraMotion, transform_box, quaternion_rotation, rotation_homography
from sar_yolo_detector.box_fusion import BoxFusion, Candidate, iou
from sar_yolo_detector.async_pipeline import FlowDetections


class CameraMotionTest(unittest.TestCase):
    def scene(self):
        gray = np.random.RandomState(17).randint(0, 240, (240, 320), dtype=np.uint8)
        gray = cv2.GaussianBlur(gray, (3, 3), .6)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def test_background_rotation_translation_and_exclusion(self):
        previous = self.scene()
        matrix = np.vstack((cv2.getRotationMatrix2D((160, 120), 3., 1.), [0, 0, 1]))
        matrix[:2, 2] += [6, 3]
        current = cv2.warpPerspective(previous, matrix, (320, 240))
        # An independently moving foreground patch must not drive GMC.
        current[70:160, 100:190] = np.random.RandomState(33).randint(0, 240, (90, 90, 3))
        motion = CameraMotion()
        self.assertIsNone(motion.update(previous, 10.))
        estimated = motion.update(current, 10.1, [(90, 60, 110, 110)])
        self.assertIsNotNone(estimated)
        self.assertEqual(motion.source, "background")
        self.assertTrue(np.allclose(estimated, matrix, atol=.6))

    def test_untextured_background_uses_pose_or_fails_closed(self):
        frame = np.full((240, 320, 3), 127, np.uint8)
        motion = CameraMotion()
        motion.update(frame, 10.)
        self.assertIsNone(motion.update(frame, 10.1))
        pose = np.array([[1., 0., 8.], [0., 1., 0.], [0., 0., 1.]])
        self.assertTrue(np.array_equal(motion.update(frame, 10.2, pose_rotation=pose), pose))
        self.assertEqual(motion.source, "pose_rotation_only")

    def test_invalid_transform_and_large_gap_rejected(self):
        frame = self.scene()
        for matrix in (np.full((3, 3), np.nan), np.diag([10., 10., 1.]),
                       np.array([[1., 0., 1000.], [0., 1., 0.], [0., 0., 1.]])):
            self.assertFalse(CameraMotion.validate(matrix, frame.shape))
        motion = CameraMotion()
        motion.update(frame, 10.)
        self.assertIsNone(motion.update(frame, 11., pose_rotation=np.eye(3)))

    def test_pose_rotation_direction_and_intrinsics(self):
        k = np.array([[300., 0, 160], [0, 300, 120], [0, 0, 1]])
        previous = np.eye(3)
        current = quaternion_rotation((0, 0, np.sin(.05), np.cos(.05)))
        matrix = rotation_homography(k, previous, current)
        ray = np.array([.2, .1, 1.])
        expected = k @ current.T @ ray
        predicted = matrix @ k @ ray
        self.assertTrue(np.allclose(predicted/predicted[2], expected/expected[2]))
        with self.assertRaises(ValueError):
            quaternion_rotation((0, 0, 0, 0))

    def test_history_is_bounded_and_never_bridges_unavailable_motion(self):
        frame = np.full((240, 320, 3), 127, np.uint8)
        motion = CameraMotion()
        for index in range(50):
            motion.update(frame, 10.+index*.01, pose_rotation=np.eye(3))
        self.assertEqual(len(motion.history), 32)
        self.assertIsNone(motion.between(10., 10.49))
        motion.update(frame, 10.5)
        self.assertIsNone(motion.between(10.49, 10.5))

    def test_camera_displacement_does_not_become_target_velocity(self):
        from test_box_fusion import LocalTracker, image
        local = LocalTracker()
        fusion = BoxFusion(tracker=local)
        old = Candidate((60, 60, 40, 40), 2, .9, 5)
        fusion.initialize(image(), old.box, 10., 99, old)
        matrix = np.array([[1., 0., 8.], [0., 1., 0.], [0., 0., 1.]])
        new = Candidate((70, 60, 40, 40), 2, .9, 5)
        local.valid = False
        result = fusion.step(image(new.box), 10.1, [new], camera_transform=matrix)
        self.assertTrue(result.observed)
        self.assertAlmostEqual(fusion.velocity[0], 8., delta=.01)  # 2px residual / .1s * .4

    def test_real_kcf_survives_camera_rotation_between_yolo_frames(self):
        from test_box_fusion import image
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("KCF unavailable")
        frame = image()
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        original = (60, 60, 40, 40)
        fusion.initialize(frame, original, 10., 99, Candidate(original, 2, .9, 5))
        previous = frame
        for index in range(1, 6):
            matrix = np.vstack((cv2.getRotationMatrix2D((130, 100), 2., 1.), [0, 0, 1]))
            current = cv2.warpPerspective(previous, matrix, (260, 200))
            result = fusion.step(current, 10.+index*.1, [], detector_updated=False, camera_transform=matrix)
            self.assertTrue(result.observed, result.reason)
            self.assertEqual(result.public_id, 99)
            original = transform_box(original, matrix)
            self.assertGreater(iou(result.box, original), .7)
            previous = current


if __name__ == "__main__":
    unittest.main()
