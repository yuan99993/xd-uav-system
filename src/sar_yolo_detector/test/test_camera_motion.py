import unittest
import cv2
import numpy as np

from sar_yolo_detector.camera_motion import CameraMotion, transform_box, quaternion_rotation, rotation_homography
from sar_yolo_detector.box_fusion import BoxFusion, Candidate, iou
from sar_yolo_detector.async_pipeline import FlowDetections


class CameraMotionTest(unittest.TestCase):
    def test_gyro_only_advance_builds_capture_transform_without_image_processing(self):
        motion=CameraMotion()
        shape=(240,320,3)
        first=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        second=np.array([[1.,0.,3.],[0.,1.,-2.],[0.,0.,1.]])
        np.testing.assert_allclose(motion.advance_imu(10.,first,shape),first)
        np.testing.assert_allclose(motion.advance_imu(10.04,second,shape),second)
        np.testing.assert_allclose(motion.between(10.,10.04),second)
        self.assertIsNone(motion.gray)
        self.assertEqual(motion.source,'optical_imu_rate')

    def test_strong_detection_can_skip_background_only_with_exact_pure_rotation(self):
        from unittest.mock import Mock
        frame=self.scene();motion=CameraMotion()
        motion.update(frame,1.)
        motion._background=Mock(return_value=np.eye(3))
        matrix=motion.update(frame,1.1,pose_rotation=np.eye(3),pure_rotation=True,estimate_background=False)
        np.testing.assert_allclose(matrix,np.eye(3))
        motion._background.assert_not_called()
        motion.update(frame,1.2,pose_rotation=np.eye(3),pure_rotation=False,estimate_background=False)
        motion._background.assert_called_once()

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

    def test_equal_measured_optical_centres_allow_exact_calibrated_pose_without_texture(self):
        frame=np.full((240,320,3),127,np.uint8);motion=CameraMotion();motion.update(frame,10.)
        pose=np.array([[1.01,0,-1.6],[0,1.01,-1.2],[0,0,1.]])
        result=motion.update(frame,10.1,pose_rotation=pose,pure_rotation=True,calibration_changed=True)
        np.testing.assert_allclose(result,pose)
        self.assertEqual(motion.source,'calibrated_pose')
        self.assertFalse(motion.background_valid)
        self.assertTrue(motion.calibration_between(10.,10.1))

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

    def test_tracking_window_does_not_inflate_under_repeated_camera_rotation(self):
        from sar_yolo_detector.camera_motion import transport_box
        initial=(100.,80.,40.,20.);box=initial
        matrix=np.vstack((cv2.getRotationMatrix2D((120.,90.),1.,1.),[0,0,1]))
        for _ in range(360):box=transport_box(box,matrix)
        np.testing.assert_allclose(box,initial,atol=1e-8)
        # A genuine optical scale change must still scale window and area.
        zoom=np.array([[2.,0.,-120.],[0.,2.,-90.],[0.,0.,1.]])
        scaled=transport_box(initial,zoom)
        self.assertAlmostEqual(scaled[2]*scaled[3],4*initial[2]*initial[3])

    def test_persistent_support_preserves_aspect_rotation_without_accumulated_envelope_growth(self):
        from sar_yolo_detector.camera_motion import transform_window
        initial=(100.,80.,60.,30.);box=initial;corners=None
        matrix=np.vstack((cv2.getRotationMatrix2D((130.,95.),1.,1.),[0,0,1]))
        for index in range(360):
            box,corners=transform_window(box,matrix,corners)
            if index==89:np.testing.assert_allclose(box[2:],[30.,60.],atol=1e-7)
        np.testing.assert_allclose(box,initial,atol=1e-7)

    def test_real_kcf_tracks_independent_target_motion_during_continuous_lens_compensation(self):
        from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
        if not hasattr(cv2,"TrackerKCF_create") and not hasattr(getattr(cv2,"legacy",object()),"TrackerKCF_create"):
            self.skipTest("KCF unavailable")
        background=np.full((480,640,3),70,np.uint8)
        patch=np.random.RandomState(77).randint(0,230,(30,40,3),np.uint8)
        def render(index,camera):
            frame=background.copy();x,y=200+2*index,140+index
            frame[y:y+30,x:x+40]=patch
            return cv2.warpPerspective(frame,camera,(640,480))
        tracker=ManualBoxTracker(dict(tracker_type='feature_tracker',feature_tracker=dict(tracker_algorithm='kcf',require_opencv_tracker=True)))
        camera=np.eye(3);previous=render(0,camera)
        self.assertTrue(tracker.initialize(previous,(200,140,40,30)).valid)
        step=np.array([[1.01,0.,-3.2],[0.,1.01,-2.4],[0.,0.,1.]])
        for index in range(1,41):
            camera=step@camera;current=render(index,camera)
            self.assertTrue(tracker.compensate_motion(previous,step,force_scale=True))
            result=tracker.update(current);self.assertTrue(result.valid,result.reason);previous=current
        expected=transform_box((280,180,40,30),camera)
        self.assertGreater(iou(result.bbox,expected),.65,repr((result.bbox,expected)))

    def test_real_kcf_accumulates_subpixel_lens_scale_without_rounding_it_away(self):
        from sar_yolo_detector.manual_box_tracker import ManualBoxTracker
        from test_box_fusion import image
        if not hasattr(cv2,"TrackerKCF_create") and not hasattr(getattr(cv2,"legacy",object()),"TrackerKCF_create"):
            self.skipTest("KCF unavailable")
        base=image((100,70,50,50));tracker=ManualBoxTracker(dict(tracker_type='feature_tracker',
            feature_tracker=dict(tracker_algorithm='kcf',require_opencv_tracker=True)))
        self.assertTrue(tracker.initialize(base,(100,70,50,50)).valid)
        matrix=np.array([[1.01,0.,-1.3],[0.,1.01,-1.],[0.,0.,1.]])
        accumulated=np.eye(3);previous=base
        for index in range(50):
            accumulated=matrix@accumulated
            current=cv2.warpPerspective(base,accumulated,(260,200))
            self.assertTrue(tracker.compensate_motion(previous,matrix,force_scale=True))
            result=tracker.update(current)
            self.assertTrue(result.valid,result.reason)
            previous=current
        expected=transform_box((100,70,50,50),accumulated)
        self.assertAlmostEqual(result.bbox[2],expected[2],delta=2.)
        self.assertGreater(iou(result.bbox,expected),.7)

    def test_real_kcf_survives_camera_rotation_between_yolo_frames(self):
        from test_box_fusion import image
        if not hasattr(cv2, "TrackerKCF_create") and not hasattr(getattr(cv2, "legacy", object()), "TrackerKCF_create"):
            self.skipTest("KCF unavailable")
        frame = image()
        fusion = BoxFusion(manual_config={"feature_tracker": {"tracker_algorithm": "kcf"}})
        original = (60, 60, 40, 40)
        fusion.initialize(frame, original, 10., 99, Candidate(original, 2, .9, 5))
        previous = frame
        initial_box = original; accumulated = np.eye(3)
        for index in range(1, 6):
            matrix = np.vstack((cv2.getRotationMatrix2D((130, 100), 2., 1.), [0, 0, 1]))
            current = cv2.warpPerspective(previous, matrix, (260, 200))
            result = fusion.step(current, 10.+index*.1, [], detector_updated=False, camera_transform=matrix)
            self.assertTrue(result.observed, result.reason)
            self.assertEqual(result.public_id, 99)
            accumulated = matrix @ accumulated
            original = transform_box(initial_box, accumulated)
            self.assertGreater(iou(result.box, original), .7)
            previous = current


if __name__ == "__main__":
    unittest.main()
