#!/usr/bin/env python3
"""Static preflight for xd_uav_track and xd_uav_detect deployment YAML.

It deliberately has no ROS-master dependency, so it can run in CI and before a
vehicle is armed.  Runtime TF availability remains checked by the detector at
the detection timestamp; this tool checks the static part of that contract.
"""

import argparse
import hashlib
import math
import os
from pathlib import Path
import sys

import yaml


class DuplicateKeyError(yaml.YAMLError):
    pass


class UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader which fails rather than silently replacing a key."""


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            where = "line %d" % (key_node.start_mark.line + 1)
            raise DuplicateKeyError("duplicate key %r at %s" % (key, where))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


class Report:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, text):
        self.errors.append(text)

    def warn(self, text):
        self.warnings.append(text)

    def require_topic(self, value, name):
        if not isinstance(value, str) or not value.strip():
            self.error("%s must be a non-empty ROS topic" % name)

    def require_frame(self, value, name, allow_empty=False):
        if allow_empty and value in (None, ""):
            return
        if not isinstance(value, str) or not value.strip() or " " in value:
            self.error("%s must be a non-empty frame id without spaces" % name)

    def finite(self, value, name, positive=False):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            self.error("%s must be finite" % name)
            return False
        if positive and value <= 0.0:
            self.error("%s must be > 0" % name)
            return False
        return True

    def vector3(self, value, name):
        if not isinstance(value, list) or len(value) != 3:
            self.error("%s must contain exactly three finite values" % name)
            return False
        return all(self.finite(item, "%s[%d]" % (name, index))
                   for index, item in enumerate(value))


def _mapping(value, name, report):
    if isinstance(value, dict):
        return value
    report.error("%s must be a mapping" % name)
    return {}


def _load(path, report):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            value = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        report.error("%s: %s" % (path, exc))
        return {}
    return _mapping(value if value is not None else {}, path, report)


def merge_mappings(base, overlay):
    """Mirror rosparam's recursive overlay behaviour without losing checks."""
    result = dict(base)
    for key, value in overlay.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = merge_mappings(result[key], value)
        else:
            result[key] = value
    return result


def validate_tracker(config, report, label):
    runtime = _mapping(config.get("runtime", {}), label + ".runtime", report)
    for key in ("maximum_vehicle_state_history_samples",
                "maximum_oosm_history_samples"):
        value = runtime.get(key)
        if not isinstance(value, int) or value < 2:
            report.error(label + ".runtime." + key + " must be an integer >= 2")
    interfaces = _mapping(config.get("interfaces", {}), label + ".interfaces", report)
    inputs = _mapping(interfaces.get("input", {}), label + ".interfaces.input", report)
    outputs = _mapping(interfaces.get("output", {}), label + ".interfaces.output", report)
    for key in ("detections", "vehicle_state"):
        report.require_topic(inputs.get(key), label + ".interfaces.input." + key)
    for key in ("control_reference", "body_velocity", "follower_command", "tracks", "status"):
        report.require_topic(outputs.get(key), label + ".interfaces.output." + key)

    tracker = _mapping(config.get("tracker", {}), label + ".tracker", report)
    sources = _mapping(tracker.get("sources", {}),
                       label + ".tracker.sources", report)
    allowed_sources = sources.get("allowed")
    if not isinstance(allowed_sources, list) or any(
            not isinstance(value, str) or not value
            for value in allowed_sources):
        report.error(label + ".tracker.sources.allowed must be a string array")
    for key in ("maximum_sources", "maximum_name_length"):
        value = sources.get(key)
        if not isinstance(value, int) or value <= 0:
            report.error(label + ".tracker.sources." + key +
                         " must be a positive integer")
    report.finite(sources.get("ttl_sec"),
                  label + ".tracker.sources.ttl_sec", positive=True)
    global_identity = _mapping(tracker.get("global_identity", {}),
                               label + ".tracker.global_identity", report)
    maximum_entities = global_identity.get("maximum_entities")
    if not isinstance(maximum_entities, int) or maximum_entities <= 0:
        report.error(label + ".tracker.global_identity.maximum_entities "
                     "must be a positive integer")
    group_reid = _mapping(tracker.get("group_reid", {}),
                          label + ".tracker.group_reid", report)
    if group_reid.get("enabled", False):
        for key in ("appearance_cosine", "trigger_score_margin",
                    "long_association_minimum_margin"):
            value = group_reid.get(key)
            if report.finite(value, label + ".tracker.group_reid." + key) and \
                    not 0.0 <= value <= 1.0:
                report.error(label + ".tracker.group_reid." + key +
                             " must be in [0, 1]")
        report.finite(group_reid.get("aspect_log_gate"),
                      label + ".tracker.group_reid.aspect_log_gate", positive=True)
        report.finite(group_reid.get("short_occlusion_sec"),
                      label + ".tracker.group_reid.short_occlusion_sec")
        for key in ("maximum_groups", "trigger_minimum_features"):
            value = group_reid.get(key)
            if not isinstance(value, int) or value <= 0:
                report.error(label + ".tracker.group_reid." + key +
                             " must be a positive integer")

    long_term = _mapping(tracker.get("long_term_identity", {}),
                         label + ".tracker.long_term_identity", report)
    if long_term.get("enabled", False):
        report.finite(long_term.get("ttl_sec"),
                      label + ".tracker.long_term_identity.ttl_sec", positive=True)
        for key in ("appearance_cosine", "minimum_margin", "maximum_cost"):
            value = long_term.get(key)
            if report.finite(value, label + ".tracker.long_term_identity." + key) and \
                    not 0.0 <= value <= 1.0:
                report.error(label + ".tracker.long_term_identity." + key +
                             " must be in [0, 1]")
        for key in ("maximum_identities", "reconfirmation_hits"):
            value = long_term.get(key)
            minimum = 2 if key == "reconfirmation_hits" else 1
            if not isinstance(value, int) or value < minimum:
                report.error(label + ".tracker.long_term_identity." + key +
                             " must be an integer >= %d" % minimum)

    identity_hints = _mapping(tracker.get("identity_hints", {}),
                              label + ".tracker.identity_hints", report)
    if identity_hints.get("enabled", False):
        report.require_topic(inputs.get("identity_hints"),
                             label + ".interfaces.input.identity_hints")
        report.finite(identity_hints.get("maximum_time_delta_sec"),
                      label + ".tracker.identity_hints.maximum_time_delta_sec",
                      positive=True)
        for key in ("minimum_confidence", "hard_confidence", "iou_gate"):
            value = identity_hints.get(key)
            if report.finite(value, label + ".tracker.identity_hints." + key) and \
                    not 0.0 <= value <= 1.0:
                report.error(label + ".tracker.identity_hints." + key +
                             " must be in [0, 1]")
        confirmations = identity_hints.get("confirmations")
        if not isinstance(confirmations, int) or confirmations < 2:
            report.error(label + ".tracker.identity_hints.confirmations must be >= 2")
        minimum = identity_hints.get("minimum_confidence")
        hard = identity_hints.get("hard_confidence")
        if isinstance(minimum, (int, float)) and isinstance(hard, (int, float)) and \
                hard < minimum:
            report.error(label + ".tracker.identity_hints.hard_confidence must be >= minimum_confidence")

    world_identity = _mapping(tracker.get("world_identity", {}),
                              label + ".tracker.world_identity", report)
    if world_identity:
        report.finite(world_identity.get("innovation_gate_sigma"),
                      label + ".tracker.world_identity.innovation_gate_sigma",
                      positive=True)
        report.finite(world_identity.get("process_noise_mps"),
                      label + ".tracker.world_identity.process_noise_mps")
    source_quality = _mapping(tracker.get("source_quality_handover", {}),
                              label + ".tracker.source_quality_handover", report)
    if source_quality.get("enabled", False):
        if tracker.get("source_policy", "auto_handover") == "fixed":
            report.error(label + ".tracker.source_quality_handover.enabled requires source_policy=auto_handover")
        for key in ("min_active_quality", "min_incoming_quality", "quality_margin"):
            value = source_quality.get(key)
            if report.finite(value, label + ".tracker.source_quality_handover." + key) and \
                    not 0.0 <= value <= 1.0:
                report.error(label + ".tracker.source_quality_handover." + key +
                             " must be in [0, 1]")
        report.finite(source_quality.get("cooldown_sec"),
                      label + ".tracker.source_quality_handover.cooldown_sec")

    camera_motion = _mapping(tracker.get("camera_motion_compensation", {}),
                             label + ".tracker.camera_motion_compensation", report)
    if camera_motion.get("enabled", False):
        for key in ("fx", "fy", "maximum_pose_delta_sec"):
            report.finite(camera_motion.get(key),
                          label + ".tracker.camera_motion_compensation." + key,
                          positive=True)
        for key in ("cx", "cy"):
            report.finite(camera_motion.get(key),
                          label + ".tracker.camera_motion_compensation." + key)
        report.vector3(camera_motion.get("optical_to_body_rpy"),
                       label + ".tracker.camera_motion_compensation.optical_to_body_rpy")
        history = camera_motion.get("pose_history_size")
        if not isinstance(history, int) or history < 2:
            report.error(label + ".tracker.camera_motion_compensation.pose_history_size must be >= 2")

    frames = _mapping(config.get("frames", {}), label + ".frames", report)
    if frames:
        report.require_frame(frames.get("body"), label + ".frames.body")

    follower = _mapping(config.get("follower", {}), label + ".follower", report)
    guidance = _mapping(follower.get("target_guidance", {}),
                        label + ".follower.target_guidance", report)
    predictive = _mapping(guidance.get("predictive_fov", {}),
                          label + ".follower.target_guidance.predictive_fov", report)
    if predictive.get("enabled", False):
        for key in ("fx", "fy", "width", "height", "horizon_sec"):
            report.finite(predictive.get(key), label + ".predictive_fov." + key,
                          positive=True)
        width, height = predictive.get("width"), predictive.get("height")
        cx, cy = predictive.get("cx"), predictive.get("cy")
        if report.finite(cx, label + ".predictive_fov.cx") and \
                isinstance(width, (int, float)) and not 0.0 <= cx <= width:
            report.error(label + ".predictive_fov.cx must lie inside image width")
        if report.finite(cy, label + ".predictive_fov.cy") and \
                isinstance(height, (int, float)) and not 0.0 <= cy <= height:
            report.error(label + ".predictive_fov.cy must lie inside image height")
        report.vector3(predictive.get("optical_to_body_rpy"),
                       label + ".predictive_fov.optical_to_body_rpy")
        report.vector3(predictive.get("translation"),
                       label + ".predictive_fov.translation")

    gimbal = _mapping(guidance.get("gimbal", {}),
                      label + ".follower.target_guidance.gimbal", report)
    if gimbal.get("dynamic_fov_enabled", False):
        margin = gimbal.get("dynamic_fov_margin_ratio")
        minimum = gimbal.get("min_fov_rad")
        if report.finite(margin, label + ".target_guidance.gimbal.dynamic_fov_margin_ratio") and \
                not 0.0 <= margin < 0.5:
            report.error(label + ".target_guidance.gimbal.dynamic_fov_margin_ratio must be in [0, 0.5)")
        report.finite(minimum, label + ".target_guidance.gimbal.min_fov_rad", positive=True)
        for key in ("zoom_reference_ratio", "zoom_margin_gain"):
            report.finite(gimbal.get(key),
                          label + ".target_guidance.gimbal." + key,
                          positive=False)

    observation = _mapping(guidance.get("observation", {}),
                           label + ".follower.target_guidance.observation", report)
    if guidance.get("enabled", False) and guidance.get("mode") in ("metric_pursuit", "metric_orbit"):
        report.finite(observation.get("metric_timeout_sec"),
                      label + ".target_guidance.observation.metric_timeout_sec", positive=True)
        report.finite(observation.get("max_position_sigma_m"),
                      label + ".target_guidance.observation.max_position_sigma_m", positive=True)

    metric = _mapping(guidance.get("metric_velocity_estimation", {}),
                      label + ".follower.target_guidance.metric_velocity_estimation", report)
    model = metric.get("world_filter_model", "cv")
    if model not in ("cv", "imm"):
        report.error(label + ".target_guidance.metric_velocity_estimation.world_filter_model must be cv or imm")
    if metric.get("world_filter_enabled", False):
        report.finite(metric.get("world_filter_process_accel_stddev_mps2"),
                      label + ".target_guidance.metric_velocity_estimation.world_filter_process_accel_stddev_mps2",
                      positive=True)
        report.finite(metric.get("world_filter_mahalanobis_gate"),
                      label + ".target_guidance.metric_velocity_estimation.world_filter_mahalanobis_gate",
                      positive=True)
        if model == "imm":
            for key in ("world_filter_imm_static_accel_stddev_mps2",
                        "world_filter_imm_maneuver_accel_stddev_mps2",
                        "world_filter_imm_turn_rate_stddev_radps2",
                        "world_filter_imm_max_turn_rate_radps"):
                report.finite(metric.get(key), label + ".target_guidance.metric_velocity_estimation." + key,
                        positive=True)
            probability = metric.get("world_filter_imm_min_probability")
            if report.finite(probability, label + ".target_guidance.metric_velocity_estimation.world_filter_imm_min_probability") and \
                    not 0.0 <= probability < 1.0:
                report.error(label + ".target_guidance.metric_velocity_estimation.world_filter_imm_min_probability must be in [0, 1)")
            for key in ("world_filter_imm_static_persistence",
                        "world_filter_imm_cv_persistence",
                        "world_filter_imm_turn_persistence"):
                persistence = metric.get(key)
                if report.finite(persistence,
                                 label + ".target_guidance.metric_velocity_estimation." + key) and \
                        not 0.0 <= persistence < 1.0:
                    report.error(label + ".target_guidance.metric_velocity_estimation." + key +
                                 " must be in [0, 1)")
    if metric.get("world_filter_oosm_enabled", False):
        report.finite(metric.get("world_filter_oosm_window_sec"),
                      label + ".target_guidance.metric_velocity_estimation.world_filter_oosm_window_sec",
                      positive=True)
        inflation = metric.get("world_filter_oosm_covariance_inflation")
        if report.finite(inflation,
                         label + ".target_guidance.metric_velocity_estimation.world_filter_oosm_covariance_inflation") and \
                inflation < 1.0:
            report.error(label + ".target_guidance.metric_velocity_estimation.world_filter_oosm_covariance_inflation must be >= 1")
        report.finite(metric.get("world_filter_pose_history_sec"),
                      label + ".target_guidance.metric_velocity_estimation.world_filter_pose_history_sec",
                      positive=True)


def validate_detector(config, report, label, strict_metric):
    interfaces = _mapping(config.get("interfaces", {}), label + ".interfaces", report)
    inputs = _mapping(interfaces.get("input", {}), label + ".interfaces.input", report)
    outputs = _mapping(interfaces.get("output", {}), label + ".interfaces.output", report)
    for key in ("detections_2d", "image", "camera_info"):
        report.require_topic(inputs.get(key), label + ".interfaces.input." + key)
    report.require_topic(outputs.get("detections_3d"), label + ".interfaces.output.detections_3d")
    localization = _mapping(config.get("localization", {}), label + ".localization", report)
    method = localization.get("method")
    frames = _mapping(config.get("frames", {}), label + ".frames", report)
    metadata = _mapping(config.get("metadata", {}), label + ".metadata", report)
    for key in ("image_source", "sensor_id", "detector_name"):
        value = metadata.get(key)
        if not isinstance(value, str) or not value.strip():
            # Keep custom legacy detector bridges usable, but make the loss of
            # source isolation visible before launch.  The launch wrapper can
            # provide these two values without editing the detector config.
            report.warn(label + ".metadata." + key +
                        " is empty; tracker source isolation relies on launch/input metadata")

    if method == "ground_plane":
        report.require_frame(frames.get("world"), label + ".frames.world")
        report.require_frame(frames.get("camera"), label + ".frames.camera", allow_empty=True)
        ground = _mapping(config.get("ground_projection", {}), label + ".ground_projection", report)
        for key in ("ground_plane_z_m", "minimum_range_m", "maximum_range_m",
                    "minimum_ray_plane_angle_deg", "pixel_stddev_px",
                    "ground_height_stddev_m", "position_stddev_m"):
            report.finite(ground.get(key), label + ".ground_projection." + key,
                          positive=key not in ("ground_plane_z_m",))
        minimum, maximum = ground.get("minimum_range_m"), ground.get("maximum_range_m")
        if isinstance(minimum, (int, float)) and isinstance(maximum, (int, float)) and minimum >= maximum:
            report.error(label + ".ground_projection minimum_range_m must be below maximum_range_m")
        angle = ground.get("minimum_ray_plane_angle_deg")
        if isinstance(angle, (int, float)) and not 0.0 < angle < 90.0:
            report.error(label + ".ground_projection minimum_ray_plane_angle_deg must be in (0, 90)")
    elif method == "gimbal_lrf":
        for key in ("gimbal_state", "gimbal_range"):
            report.require_topic(inputs.get(key), label + ".interfaces.input." + key)
        gimbal = _mapping(config.get("gimbal_range", {}), label + ".gimbal_range", report)
        calibrated = gimbal.get("calibrated", False)
        if not isinstance(calibrated, bool):
            report.error(label + ".gimbal_range.calibrated must be boolean")
        elif not calibrated:
            (report.error if strict_metric else report.warn)(
                label + ".gimbal_range.calibrated is false; metric LRF output is intentionally disabled")
        for key in ("minimum_range_m", "maximum_range_m", "maximum_time_difference_sec",
                    "range_stddev_m", "yaw_stddev_deg", "pitch_stddev_deg",
                    "mount_angle_stddev_deg", "mount_translation_stddev_m"):
            report.finite(gimbal.get(key), label + ".gimbal_range." + key, positive=True)
        for key in ("range_time_offset_sec", "gimbal_time_offset_sec"):
            report.finite(gimbal.get(key), label + ".gimbal_range." + key)
        report.vector3(gimbal.get("mount_translation_body_frd"),
                       label + ".gimbal_range.mount_translation_body_frd")
        report.vector3(gimbal.get("mount_rotation_ypr"),
                       label + ".gimbal_range.mount_rotation_ypr")
        minimum, maximum = gimbal.get("minimum_range_m"), gimbal.get("maximum_range_m")
        if isinstance(minimum, (int, float)) and isinstance(maximum, (int, float)) and minimum >= maximum:
            report.error(label + ".gimbal_range minimum_range_m must be below maximum_range_m")
    elif method == "lidar_camera":
        report.require_topic(inputs.get("point_cloud"), label + ".interfaces.input.point_cloud")
    else:
        report.error(label + ".localization.method must be lidar_camera, ground_plane, or gimbal_lrf")


def _resolve_package_resource(value, report, label):
    if not isinstance(value, str) or not value.strip():
        report.error(label + " must be a non-empty model path")
        return None
    value = value.strip()
    if not value.startswith("package://"):
        return Path(value).expanduser()
    package_and_path = value[len("package://"):]
    package, separator, relative = package_and_path.partition("/")
    if not package or not separator or not relative:
        report.error(label + " has an invalid package URI")
        return None
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(":"):
        candidate = Path(root) / package
        if candidate.is_dir():
            return candidate / relative
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path(package)) / relative
    except Exception:
        report.error(label + " cannot resolve package " + package)
        return None


def _validate_reid_model(profile, backend, report, label):
    path = _resolve_package_resource(profile.get("model_path"), report,
                                     label + ".model_path")
    if path is None:
        return
    if path.is_symlink():
        report.error(label + ".model_path must not be a symbolic link")
        return
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        report.error(label + ".model_path: " + str(error))
        return
    if not resolved.is_file():
        report.error(label + ".model_path must be a regular file")
        return
    expected_suffix = ".onnx" if backend == "onnx" else ".pt"
    if resolved.suffix.lower() != expected_suffix:
        report.error(label + ".model_path must use " + expected_suffix)
    expected = str(profile.get("sha256", "") or "").strip().lower()
    require_digest = bool(profile.get("require_sha256", True))
    if require_digest and (len(expected) != 64 or
                           any(character not in "0123456789abcdef"
                               for character in expected)):
        report.error(label + ".sha256 must contain 64 hexadecimal characters")
        return
    if expected:
        digest = hashlib.sha256()
        try:
            with resolved.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
        except OSError as error:
            report.error(label + ".model_path: " + str(error))
            return
        if digest.hexdigest() != expected:
            report.error(label + ".sha256 does not match the model file")


def validate_reid(config, report, label, requested_profile=""):
    root = _mapping(config.get("reid", config), label + ".reid", report)
    synchronization = _mapping(root.get("synchronization", {}),
                               label + ".reid.synchronization", report)
    runtime = _mapping(root.get("runtime", {}), label + ".reid.runtime", report)
    for key in ("maximum_image_delta_sec", "maximum_image_wait_sec"):
        value = synchronization.get(key)
        if report.finite(value, label + ".reid.synchronization." + key) and value < 0.0:
            report.error(label + ".reid.synchronization." + key + " must be >= 0")
    for key in ("image_cache_size",):
        value = synchronization.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or value < 2:
            report.error(label + ".reid.synchronization." + key + " must be an integer >= 2")
    for key in ("maximum_rois_per_frame",):
        value = runtime.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            report.error(label + ".reid.runtime." + key + " must be an integer >= 1")
    report.finite(runtime.get("maximum_output_age_sec"),
                  label + ".reid.runtime.maximum_output_age_sec", positive=True)

    profiles = _mapping(root.get("model_profiles", {}),
                        label + ".reid.model_profiles", report)
    active = str(requested_profile or root.get("active_model_profile", "") or "").strip()
    if not active:
        report.error(label + ".reid.active_model_profile must be non-empty")
        return
    selected = profiles.get(active)
    if not isinstance(selected, dict):
        report.error(label + ".reid.model_profiles has no profile " + active)
        return
    class_profiles = _mapping(selected.get("class_profiles", {}),
                              label + ".reid.model_profiles." + active +
                              ".class_profiles", report)
    if not class_profiles:
        report.error(label + ".reid profile " + active + " has no class profiles")
        return
    allowed_backends = {"disabled", "none", "histogram", "hybrid", "deep", "onnx"}
    allowed_fallbacks = {"disabled", "none", "histogram", "hybrid"}
    owned_classes = {}
    for name, raw_profile in class_profiles.items():
        profile_label = (label + ".reid.model_profiles." + active +
                         ".class_profiles." + str(name))
        profile = _mapping(raw_profile, profile_label, report)
        backend = str(profile.get("backend", root.get("default_backend", "hybrid"))).lower()
        if backend not in allowed_backends:
            report.error(profile_label + ".backend is unsupported: " + backend)
        fallback = str(profile.get("fallback_backend", "none") or "none").lower()
        if fallback not in allowed_fallbacks:
            report.error(profile_label + ".fallback_backend is unsupported: " + fallback)
        association_weight = profile.get("association_weight", 1.0)
        if (report.finite(association_weight,
                          profile_label + ".association_weight") and
                not 0.0 <= association_weight <= 1.0):
            report.error(profile_label + ".association_weight must be in [0, 1]")
        class_ids = profile.get("class_ids")
        if not isinstance(class_ids, list) or not class_ids:
            report.error(profile_label + ".class_ids must be a non-empty list")
            continue
        for raw_class_id in class_ids:
            if not isinstance(raw_class_id, int) or isinstance(raw_class_id, bool) or raw_class_id < 0:
                report.error(profile_label + ".class_ids must contain non-negative integers")
                continue
            previous = owned_classes.get(raw_class_id)
            if previous is not None:
                report.error("class id %d belongs to both %s and %s" %
                             (raw_class_id, previous, name))
            owned_classes[raw_class_id] = name
        if backend in ("deep", "onnx"):
            _validate_reid_model(profile, backend, report, profile_label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-config", action="append", default=[], metavar="YAML")
    parser.add_argument("--detector-config", action="append", default=[], metavar="YAML")
    parser.add_argument("--reid-config", action="append", default=[], metavar="YAML")
    parser.add_argument("--reid-profile", default="", metavar="NAME")
    parser.add_argument("--strict-metric", action="store_true",
                        help="reject uncalibrated metric detector configurations")
    args = parser.parse_args()
    if not args.track_config and not args.detector_config and not args.reid_config:
        parser.error("at least one track, detector, or ReID config is required")
    report = Report()
    tracker_config = {}
    for path in args.track_config:
        tracker_config = merge_mappings(tracker_config, _load(path, report))
    if args.track_config:
        validate_tracker(tracker_config, report,
                         " + ".join(args.track_config))
    for path in args.detector_config:
        validate_detector(_load(path, report), report, path, args.strict_metric)
    reid_config = {}
    for path in args.reid_config:
        reid_config = merge_mappings(reid_config, _load(path, report))
    if args.reid_config:
        validate_reid(reid_config, report, " + ".join(args.reid_config),
                      args.reid_profile)
    for text in report.warnings:
        print("WARNING: " + text, file=sys.stderr)
    for text in report.errors:
        print("ERROR: " + text, file=sys.stderr)
    if report.errors:
        return 2
    print("Configuration preflight passed (%d tracker, %d detector, %d ReID file(s))." %
          (len(args.track_config), len(args.detector_config), len(args.reid_config)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
