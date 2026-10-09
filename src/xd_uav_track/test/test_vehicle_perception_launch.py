#!/usr/bin/env python3
"""Check the production launch graph without starting Gazebo or a YOLO model."""

import subprocess
import unittest

import yaml


def launch_output(command, package, launch_file, *arguments):
    result = subprocess.run(
        ["roslaunch", command, package, launch_file, *arguments],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=20)
    return result.stdout


class VehiclePerceptionLaunchTest(unittest.TestCase):
    def test_portable_core_is_perception_only_and_namespace_neutral(self):
        nodes = launch_output("--nodes", "xd_uav_perception_bringup",
                              "perception.launch", "UAV_NAME:=portable7").splitlines()
        self.assertEqual({"/portable7/track", "/portable7/perception_mode_manager",
                          "/portable7/sar_yolo_multi_source_detection",
                          "/portable7/fixed_perception_overlay", "/portable7/gimbal_perception_overlay"}, set(nodes))
        params = yaml.safe_load(launch_output("--dump-params", "xd_uav_perception_bringup",
                                             "perception.launch", "UAV_NAME:=portable7"))
        self.assertTrue(params["/portable7/track/runtime/perception_only"])
        self.assertFalse(params["/portable7/track/runtime/publish_control_reference"])
        self.assertFalse(params["/portable7/track/tracker/enabled_at_startup"])
        self.assertEqual("detect", params["/portable7/perception_mode_manager/initial_mode"])
        self.assertEqual("portable7/base_link_frd", params["/portable7/track/frames/body_frd"])
        self.assertEqual("", params["/portable7/sar_yolo_multi_source_detection/engine_path"])
        self.assertFalse(any("gazebo" in name or "moving_uav_search_driver" in name for name in nodes))

    def test_dual_camera_simulation_uses_consistent_metric_anchor(self):
        params = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
            "moving_uav_vehicle_search_demo.launch", "enable_gimbal_camera:=true"))
        roots = ["/moving_uav_search/" + name + "_camera_ground_projection/ground_projection/"
                 for name in ("fixed", "gimbal")]
        self.assertEqual(params[roots[0] + "box_anchor_y_ratio"],
                         params[roots[1] + "box_anchor_y_ratio"])
        for root in roots:
            self.assertEqual(1.0, params[root + "position_stddev_m"])
        self.assertEqual(1.25, params[
            "/moving_uav_search/track/tracker/tracklet_stitching/maximum_confirmation_gap_sec"])

    def test_web_video_runs_without_desktop_compositor(self):
        arguments = ("UAV_NAME:=uav7", "enable_perception_modes:=true",
                     "enable_tracking_visualization:=false",
                     "web_video_rate_hz:=5", "perception_frame_cache_size:=30")
        nodes = launch_output("--nodes", "xd_uav_track",
                              "vision_tracking_stack.launch", *arguments).splitlines()
        self.assertIn("/uav7/fixed_perception_overlay", nodes)
        self.assertIn("/uav7/gimbal_perception_overlay", nodes)
        self.assertNotIn("/uav7/combined_tracking_view", nodes)
        params = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
                                             "vision_tracking_stack.launch", *arguments))
        self.assertEqual(30, params["/uav7/perception_mode_manager/frame_cache_size"])
        for source in ("fixed", "gimbal"):
            root = "/uav7/" + source + "_perception_overlay/"
            self.assertTrue(params[root + "enable_web_video"])
            self.assertEqual(5.0, params[root + "video_rate_hz"])

    def test_web_app_has_bounded_public_transport_and_custom_namespace(self):
        params = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
            "perception_web_app.launch", "UAV_NAME:=uav7", "http_port:=18080",
            "websocket_port:=19090"))
        self.assertEqual(18080, params["/uav7/perception_web_http/port"])
        self.assertEqual(19090, params["/uav7/perception_web_http/websocket_port"])
        self.assertEqual("uav7", params["/uav7/perception_web_http/uav_name"])
        self.assertEqual(33554432, params["/rosbridge_websocket/max_message_size"])
        self.assertIn("/uav7/perception/video", params["/rosbridge_websocket/topics_glob"])
        self.assertNotIn("/uav7/track", params["/rosbridge_websocket/topics_glob"])
        numeric = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
            "perception_web_bridge.launch", "UAV_NAME:=uav7", "enable_video:=false"))
        self.assertNotIn("/uav7/perception/video", numeric["/rosbridge_websocket/topics_glob"])

    def test_frd_frame_is_shared_by_tracker_and_mode_manager_for_both_vehicles(self):
        for vehicle in ("multirotor", "fixedwing"):
            params = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
                "vision_tracking_stack.launch", "UAV_NAME:=uav7", "vehicle_type:=" + vehicle,
                "enable_perception_modes:=true", "body_frd_frame:=uav7/custom_body_frd"))
            self.assertEqual("uav7/custom_body_frd", params["/uav7/track/frames/body_frd"])
            self.assertEqual("uav7/custom_body_frd", params["/uav7/perception_mode_manager/body_frd_frame"])

    def test_single_production_entry_owns_one_detector(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "vision_tracking_stack.launch", "enable_perception_modes:=true").splitlines()
        self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                for n in nodes))
        self.assertIn("/uav1/detect_fixed", nodes)
        self.assertIn("/uav1/detect_gimbal_lrf", nodes)
        self.assertIn("/uav1/track", nodes)
        self.assertIn("/uav1/combined_tracking_view", nodes)
        self.assertIn("/uav1/perception_mode_manager", nodes)
        self.assertIn("/uav1/fixed_perception_overlay", nodes)
        self.assertIn("/uav1/gimbal_perception_overlay", nodes)
        self.assertFalse(any("smart_tracker" in n for n in nodes))

    def test_new_modes_release_gate_and_initial_mode_are_explicit(self):
        nodes = launch_output("--nodes", "xd_uav_track", "vision_tracking_stack.launch").splitlines()
        self.assertNotIn("/uav1/perception_mode_manager", nodes)
        params = yaml.safe_load(launch_output("--dump-params", "xd_uav_track",
            "vision_tracking_stack.launch", "enable_perception_modes:=true", "initial_perception_mode:=fusion"))
        self.assertEqual("fusion", params["/uav1/perception_mode_manager/initial_mode"])

    def test_detection_only_uses_same_runtime_without_tracker(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "vision_tracking_stack.launch",
                              "enable_tracking:=false").splitlines()
        self.assertEqual(["/uav1/sar_yolo_multi_source_detection"], nodes)
        params = yaml.safe_load(launch_output(
            "--dump-params", "xd_uav_track", "vision_tracking_stack.launch",
            "enable_tracking:=false"))
        root = "/uav1/sar_yolo_multi_source_detection/"
        self.assertEqual("tensorrt_fp16", params[root + "inference_backend"])
        self.assertFalse(params[root + "pause_without_subscribers"])
        self.assertEqual("", params[root + "tracking_tracks_topic"])
        self.assertFalse(params[root + "sources"][0]["reid_enabled"])

    def test_dual_camera_still_shares_one_model_and_tracker(self):
        nodes = launch_output("--nodes", "xd_uav_track",
                              "tracking_fixedwing_dual_sensor.launch").splitlines()
        self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                for n in nodes))
        self.assertEqual(1, sum(n.endswith("/track") for n in nodes))
        self.assertIn("/uav1/detect_fixed", nodes)
        self.assertIn("/uav1/detect_gimbal_lrf", nodes)
        self.assertEqual(1, sum(n.endswith("/combined_tracking_view")
                                for n in nodes))

    def test_demo_search_prioritizes_fixed_and_follow_prefers_gimbal(self):
        params = yaml.safe_load(launch_output(
            "--dump-params", "xd_uav_track",
            "moving_uav_vehicle_search_gimbal_demo.launch",
            "gui:=false", "show_combined_overlay:=false"))
        root = "/moving_uav_search/sar_yolo_multi_source_detection/"
        self.assertEqual("fixed", params[root + "search_priority_source"])
        self.assertEqual("gimbal", params[root + "preferred_tracking_source"])
        self.assertEqual("track/tracks_by_source",
                         params[root + "tracking_tracks_topic"])
        fixed, gimbal = params[root + "sources"]
        self.assertEqual(4.0, fixed["discovery_max_fps"])
        self.assertEqual(2.0, gimbal["discovery_max_fps"])

    def test_camera_specific_compatibility_entries_keep_one_model(self):
        cases = (
            ("tracking_multirotor_fixed.launch", "detect_fixed", "detect_gimbal_lrf"),
            ("tracking_multirotor_gimbal_lrf.launch", "detect_gimbal_lrf", "detect_fixed"),
            ("tracking_fixedwing_fixed.launch", "detect_fixed", "detect_gimbal_lrf"),
        )
        for launch_file, wanted, unwanted in cases:
            with self.subTest(launch_file=launch_file):
                nodes = launch_output("--nodes", "xd_uav_track", launch_file).splitlines()
                self.assertEqual(1, sum("sar_yolo_multi_source_detection" in n
                                        for n in nodes))
                self.assertEqual(1, sum(n.endswith("/track") for n in nodes))
                self.assertIn("/uav1/" + wanted, nodes)
                self.assertNotIn("/uav1/" + unwanted, nodes)

    def test_standalone_vision_output_needs_no_xd_track_topic(self):
        params = yaml.safe_load(launch_output(
            "--dump-params", "sar_yolo_detector", "detection_only.launch"))
        root = "/sar_yolo_detection_only/"
        self.assertEqual("tensorrt_fp16", params[root + "inference_backend"])
        source, = params[root + "sources"]
        self.assertEqual("", source["xd_detections_topic"])
        self.assertTrue(source["publish_vision_detections"])
        self.assertFalse(source["reid_enabled"])


if __name__ == "__main__":
    unittest.main()
