#!/usr/bin/env python3
"""TCP-controlled launcher for the ROS processes used by the UAV stack."""

import argparse
import copy
import fcntl
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

import yaml


_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_ARG_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_PLACEHOLDER_RE = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")


def _read_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("YAML root must be a mapping: {}".format(path))
    return value


def _proc_info(pid):
    """Return (process_group, start_time_ticks) for a live Linux process."""
    try:
        raw = Path("/proc/{}/stat".format(int(pid))).read_text(encoding="utf-8")
        fields = raw[raw.rfind(")") + 2:].split()
        return int(fields[2]), fields[19]
    except (OSError, ValueError, IndexError):
        return None, None


def _expand_string(value, context):
    def replace(match):
        key = match.group(1)
        if key not in context:
            raise ValueError("unknown launch placeholder {{{}}}".format(key))
        return str(context[key])
    return _PLACEHOLDER_RE.sub(replace, str(value))


def _expand_tree(value, context):
    if isinstance(value, str):
        return _expand_string(value, context)
    if isinstance(value, list):
        return [_expand_tree(item, context) for item in value]
    if isinstance(value, dict):
        return {key: _expand_tree(item, context) for key, item in value.items()}
    return copy.deepcopy(value)


class ProcessManager:
    def __init__(self, config_path):
        self.config_path = Path(config_path).expanduser().resolve()
        self.config_dir = self.config_path.parent
        self.config = _read_yaml(self.config_path)
        self.workspace = Path(os.path.expandvars(os.path.expanduser(
            self.config.get("workspace", str(self.config_dir.parent.parent))
        ))).resolve()

        network = self.config.get("network", {})
        self.host = str(network.get("host", "0.0.0.0"))
        self.port = int(network.get("port", 8765))
        self.timeout_sec = float(network.get("heartbeat_timeout_sec", 30.0))
        self.max_message_bytes = int(network.get("max_message_bytes", 1024 * 1024))
        self.auth_token_env = str(network.get("auth_token_env", ""))
        self.auth_token = os.environ.get(self.auth_token_env, "") if self.auth_token_env else ""
        self.require_auth = bool(network.get("require_auth", True))
        if self.require_auth and not self.auth_token:
            raise RuntimeError("set {} before starting the listener".format(
                self.auth_token_env or "network.auth_token_env"))
        self.max_vehicles = int(self.config.get("max_vehicles", 60))
        self.setup_scripts = [
            _expand_string(os.path.expandvars(os.path.expanduser(str(item))),
                           {"workspace": str(self.workspace)})
            for item in self.config.get("setup_scripts", [])
        ]

        self.runtime_dir = Path(os.path.expandvars(os.path.expanduser(
            str(self.config.get("runtime_dir", "~/.ros/xd_uav_start_manager"))
        ))).resolve()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        os.chmod(self.runtime_dir, 0o700)
        self.registry_path = self.runtime_dir / "processes.json"
        self.lock_stream = open(self.runtime_dir / "manager.lock", "a+", encoding="utf-8")
        try:
            fcntl.flock(self.lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another start manager already owns {}".format(
                self.runtime_dir)) from error

        self.active = {}
        self.session_id = None
        self.generation_dir = None
        self.shutdown_event = threading.Event()
        self.server_socket = None
        self._recover_previous_processes()

    def _log(self, message):
        print("[start-manager] " + message, flush=True)

    def _load_registry(self):
        try:
            value = json.loads(self.registry_path.read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def _write_registry(self):
        data = {
            "session_id": self.session_id,
            "generation_dir": str(self.generation_dir) if self.generation_dir else None,
            "processes": {key: proc["record"] for key, proc in self.active.items()},
        }
        temp_path = self.registry_path.with_suffix(".json.tmp")
        temp_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        os.chmod(temp_path, 0o600)
        os.replace(temp_path, self.registry_path)

    def _signal_records(self, records):
        """Immediately SIGKILL all recorded groups after checking PID identity."""
        signalled = set()
        for record in records:
            try:
                pid = int(record.get("pid", -1))
                pgid = int(record.get("pgid", -1))
                expected_start = str(record.get("start_time", ""))
            except (AttributeError, TypeError, ValueError):
                self._log("skipping malformed process record")
                continue
            if pid <= 1 or pgid != pid or pgid in signalled:
                continue

            actual_pgid, actual_start = _proc_info(pid)
            if actual_start is not None and (
                    actual_pgid != pgid or actual_start != expected_start):
                self._log("skipping PID {} because its identity changed".format(pid))
                continue
            if actual_start is None:
                try:
                    os.killpg(pgid, 0)
                except ProcessLookupError:
                    continue
                except PermissionError:
                    self._log("permission denied checking process group {}".format(pgid))
                    continue

            try:
                os.killpg(pgid, signal.SIGKILL)
                signalled.add(pgid)
            except ProcessLookupError:
                continue
            except PermissionError:
                self._log("permission denied killing process group {}".format(pgid))
        return signalled

    def _recover_previous_processes(self):
        registry = self._load_registry()
        records = registry.get("processes", {})
        if isinstance(records, dict) and records:
            self._log("stopping {} process group(s) left by the previous manager".format(
                len(records)))
            self._signal_records(list(records.values()))
        self.registry_path.unlink(missing_ok=True)
        generated_root = self.runtime_dir / "generated"
        if generated_root.exists():
            shutil.rmtree(generated_root)
        (self.runtime_dir / "generated").mkdir(mode=0o700, exist_ok=True)

    def _new_generation_dir(self):
        if self.generation_dir is None:
            path = self.runtime_dir / "generated" / uuid.uuid4().hex
            path.mkdir(parents=True, mode=0o700)
            self.generation_dir = path
        return self.generation_dir

    def _write_generated_yaml(self, name, value):
        content = yaml.safe_dump(value, sort_keys=False, allow_unicode=True)
        digest = hashlib.sha256(content.encode("utf-8")).hexdigest()[:12]
        path = self._new_generation_dir() / "{}-{}.yaml".format(name, digest)
        if not path.exists():
            path.write_text(content, encoding="utf-8")
            os.chmod(path, 0o600)
        return str(path)

    def _profile_path(self, value):
        path = Path(os.path.expandvars(os.path.expanduser(str(value))))
        if not path.is_absolute():
            path = self.config_dir / path
        return str(path.resolve())

    def _fleet_context(self, profile_name, profile, vehicles, shared_launches):
        base_context = {"workspace": str(self.workspace)}
        context = _expand_tree(copy.deepcopy(self.config.get("parameters", {})),
                               base_context)
        context.update(_expand_tree(copy.deepcopy(profile.get("parameters", {})),
                                     base_context))
        context.update({
            "profile": profile_name,
            "workspace": str(self.workspace),
        })
        files = profile.get("files", {})
        world_config = None
        formation_config = None

        if "world_tf" in shared_launches:
            source = self._profile_path(files["world_tf_template"])
            world_config = _read_yaml(source)
            names = [vehicle["name"] for vehicle in vehicles]
            world_config["vehicles"] = names
            old_vehicle_configs = world_config.get("vehicle_configs", {})
            defaults = profile.get("world_vehicle_defaults", {
                "enabled": True,
                "mode": "gps_origin",
                "origin_topic": "/{name}/mavros/global_position/gp_origin_lla",
                "local_origin_frame": "{name}/local_origin",
                "position_offset": [0.0, 0.0, 0.0],
                "yaw_offset": 0.0,
            })
            new_vehicle_configs = {}
            for vehicle in vehicles:
                name = vehicle["name"]
                existing = old_vehicle_configs.get(name)
                if existing is None:
                    existing = _expand_tree(defaults, {"name": name})
                new_vehicle_configs[name] = existing
            world_config["vehicle_configs"] = new_vehicle_configs
            context["world_tf_config"] = self._write_generated_yaml(
                "world_tf", world_config)

        if "formation" in shared_launches:
            if len(vehicles) < 2:
                raise ValueError("formation task requires at least two vehicles")
            source = self._profile_path(files["formation_template"])
            formation_config = _read_yaml(source)
            names = [vehicle["name"] for vehicle in vehicles]
            formation_config["team_id"] = profile_name
            for mode in ("leader_follow", "consensus"):
                section = formation_config.get(mode)
                if not isinstance(section, dict):
                    continue
                if mode == "consensus":
                    section["team_id"] = profile_name + "_consensus"
                    section["members"] = names
                else:
                    leader = section.get("leader_name", names[0])
                    if leader not in names:
                        leader = names[0]
                        section["leader_name"] = leader
                    members = [name for name in names if name != leader]
                    section["members"] = members
                    spacing = float(profile.get("formation_longitudinal_spacing", 80.0))
                    lateral = float(profile.get("formation_lateral_offset", 40.0))
                    configured_offsets = section.get("offsets", {})
                    offsets = {name: copy.deepcopy(configured_offsets[name])
                               for name in members if name in configured_offsets}
                    for index, name in enumerate(members):
                        if name in offsets:
                            continue
                        row = index // 2 + 1
                        side = 1.0 if index % 2 == 0 else -1.0
                        offsets[name] = [-spacing * row, side * lateral, 0.0]
                    section["offsets"] = offsets
            context["formation_config"] = self._write_generated_yaml(
                "formation", formation_config)

        for type_name, path in files.get("tf_config_by_type", {}).items():
            context["tf_config_" + str(type_name)] = self._profile_path(path)
        for type_name, path in files.get("controller_config_by_type", {}).items():
            context["controller_config_" + str(type_name)] = self._profile_path(path)

        for key, value in files.items():
            if (key.endswith("_config") or key.endswith("_config_template")) and isinstance(value, str):
                context[key] = self._profile_path(value)
        return context

    def _generate_sar_overlay(self, context):
        template = context.get("sar_vehicle_config_template")
        if not template:
            raise ValueError("selected profile has no sar_vehicle_config_template")
        overlay = _read_yaml(template)
        dynamic_values = (
            "input_image_topic", "camera_info_topic", "tracked_detections_topic",
            "fusion_detections_topic", "fusion_observations_topic", "state_topic",
            "annotated_image_topic", "selected_detections_topic", "select_pixel_topic",
            "auto_select_service", "auto_select_status_topic", "gimbal_detection_topic",
            "bridge_input_topic", "bridge_input_type", "output_detections_topic",
            "detect_bridge_enabled", "detect_output_detections_topic", "uav_id",
            "sensor_id", "profile", "model_version", "calibration_version",
            "image_source", "camera_motion_fixed_frame", "image_width", "image_height",
        )
        for key in dynamic_values:
            if key in context:
                overlay[key] = context[key]
        return self._write_generated_yaml(
            "sar-{}-{}".format(context["name"], context["sensor_instance"]), overlay)

    def _generate_track_config(self, template_path, vehicle_name, sar_sensor_context,
                               fallback_image_topic=None):
        track_config = _read_yaml(template_path)
        overlay = track_config.setdefault("phase_overlay", {})
        if sar_sensor_context:
            overlay["input_image_topic"] = sar_sensor_context["annotated_image_topic"]
            overlay["input_detections_topic"] = "track/selected_detections"
        elif fallback_image_topic:
            overlay["enabled"] = True
            overlay["input_image_topic"] = fallback_image_topic
        else:
            overlay["enabled"] = False
        return self._write_generated_yaml("track-{}".format(vehicle_name), track_config)

    def _vehicle_launch_spec(self, vehicle_type, launch_id):
        type_specs = self.config.get("aircraft_types", {}).get(
            vehicle_type, {}).get("launches", [])
        for spec in type_specs:
            if isinstance(spec, dict) and spec.get("id") == launch_id:
                return spec
        return self.config.get("vehicle_launches", {}).get(launch_id)

    @staticmethod
    def _normalise_sensors(raw_sensors):
        if isinstance(raw_sensors, dict):
            iterable = [dict(value or {}, id=key)
                        for key, value in raw_sensors.items()]
        elif isinstance(raw_sensors, list):
            iterable = raw_sensors
        else:
            raise ValueError("vehicle sensors must be a list or a mapping")

        result = []
        seen = set()
        for item in iterable:
            if not isinstance(item, dict):
                raise ValueError("each sensor must be an object with id and type")
            sensor_id = item.get("id", item.get("name"))
            sensor_type = item.get("type")
            params = item.get("params", {})
            if not isinstance(sensor_id, str) or not _ARG_NAME_RE.fullmatch(sensor_id):
                raise ValueError("invalid sensor id: {}".format(sensor_id))
            if not isinstance(sensor_type, str) or not _NAME_RE.fullmatch(sensor_type):
                raise ValueError("invalid sensor type: {}".format(sensor_type))
            if sensor_id in seen:
                raise ValueError("duplicate sensor id {}".format(sensor_id))
            if not isinstance(params, dict):
                raise ValueError("sensor params must be a mapping")
            for key, value in params.items():
                if not isinstance(key, str) or not _ARG_NAME_RE.fullmatch(key):
                    raise ValueError("invalid parameter name for sensor {}: {}".format(
                        sensor_id, key))
                if not isinstance(value, (str, int, float, bool)):
                    raise ValueError("sensor parameter {} on {} must be scalar".format(
                        key, sensor_id))
            seen.add(sensor_id)
            result.append({
                "id": sensor_id,
                "name": sensor_id,
                "type": sensor_type,
                "params": params,
            })
        return result

    def _normalise_vehicle_launches(self, raw_launches, vehicle_type, sensors,
                                    vehicle_name):
        if not isinstance(raw_launches, list):
            raise ValueError("launches for {} must be a list".format(vehicle_name))
        sensor_by_id = {sensor["id"]: sensor for sensor in sensors}
        result = []
        seen = set()
        always_started = set(self.config.get("required_vehicle_launches", []))
        for item in raw_launches:
            if isinstance(item, str):
                launch_id, sensor_id, params = item, None, {}
            elif isinstance(item, dict):
                launch_id = item.get("id")
                sensor_id = item.get("sensor")
                params = item.get("params", {})
            else:
                raise ValueError("each launch for {} must be a name or object".format(
                    vehicle_name))
            if not isinstance(launch_id, str) or not _NAME_RE.fullmatch(launch_id):
                raise ValueError("invalid launch id for {}: {}".format(
                    vehicle_name, launch_id))
            if launch_id in always_started:
                raise ValueError("launch {} starts automatically for every aircraft; "
                                 "remove it from {}.launches".format(
                                     launch_id, vehicle_name))
            spec = self._vehicle_launch_spec(vehicle_type, launch_id)
            if not isinstance(spec, dict):
                raise ValueError("unknown launch {} for {} ({})".format(
                    launch_id, vehicle_name, vehicle_type))
            if sensor_id is not None:
                if not isinstance(sensor_id, str) or sensor_id not in sensor_by_id:
                    raise ValueError("launch {} on {} refers to unknown sensor {}".format(
                        launch_id, vehicle_name, sensor_id))
                allowed_types = spec.get("sensor_types", [])
                if allowed_types and sensor_by_id[sensor_id]["type"] not in allowed_types:
                    raise ValueError("launch {} cannot use sensor {} of type {}".format(
                        launch_id, sensor_id, sensor_by_id[sensor_id]["type"]))
            elif spec.get("sensor_required", False):
                raise ValueError("launch {} on {} requires a sensor id".format(
                    launch_id, vehicle_name))
            if not isinstance(params, dict):
                raise ValueError("params for launch {} on {} must be a mapping".format(
                    launch_id, vehicle_name))
            for key, value in params.items():
                if not isinstance(key, str) or not _ARG_NAME_RE.fullmatch(key):
                    raise ValueError("invalid parameter name for launch {}: {}".format(
                        launch_id, key))
                if not isinstance(value, (str, int, float, bool)):
                    raise ValueError("launch parameter {} on {} must be scalar".format(
                        key, vehicle_name))
            missing = sorted(set(spec.get("required_params", [])) - set(params))
            if missing:
                raise ValueError("launch {} on {} requires parameter(s): {}".format(
                    launch_id, vehicle_name, ", ".join(missing)))
            for key, choices in spec.get("parameter_choices", {}).items():
                if key in params and params[key] not in choices:
                    raise ValueError("parameter {} for launch {} on {} must be one of: {}".format(
                        key, launch_id, vehicle_name, ", ".join(choices)))
            for selector, requirements in spec.get("conditional_required_params", {}).items():
                selected_value = str(params.get(selector, ""))
                missing = sorted(set(requirements.get(selected_value, [])) - set(params))
                if missing:
                    raise ValueError(
                        "launch {} on {} with {}={} requires parameter(s): {}".format(
                            launch_id, vehicle_name, selector, selected_value,
                            ", ".join(missing)))
            if sensor_id is not None:
                required_by_type = spec.get("required_sensor_params", {})
                required_sensor_params = required_by_type.get(
                    sensor_by_id[sensor_id]["type"], [])
                missing = sorted(set(required_sensor_params) -
                                 set(sensor_by_id[sensor_id]["params"]))
                if missing:
                    raise ValueError("launch {} on {} needs sensor {} parameter(s): {}".format(
                        launch_id, vehicle_name, sensor_id, ", ".join(missing)))
            identity = (launch_id, sensor_id or "")
            if identity in seen:
                raise ValueError("duplicate launch {} for {} and sensor {}".format(
                    launch_id, vehicle_name, sensor_id or "(none)"))
            seen.add(identity)
            result.append({"id": launch_id, "sensor": sensor_id, "params": params})
        return result

    def _validate_snapshot(self, message):
        if "tasks" in message:
            raise ValueError(
                "top-level tasks are no longer accepted; use shared_launches and "
                "each vehicle's launches list")
        session_id = message.get("session_id")
        if not isinstance(session_id, str) or not _NAME_RE.fullmatch(session_id):
            raise ValueError("session_id must contain only letters, digits, _ or -")
        profile_name = message.get("profile", self.config.get("default_profile", ""))
        profiles = self.config.get("profiles", {})
        if profile_name not in profiles:
            raise ValueError("unknown profile: {}".format(profile_name))

        raw_shared = message.get("shared_launches", [])
        if not isinstance(raw_shared, list) or any(
                not isinstance(value, str) for value in raw_shared):
            raise ValueError("shared_launches must be a list of configured launch ids")
        if len(set(raw_shared)) != len(raw_shared):
            raise ValueError("shared_launches contains duplicates")
        shared_specs = self.config.get("shared_launches", {})
        unknown_shared = sorted(set(raw_shared) - set(shared_specs))
        if unknown_shared:
            raise ValueError("unknown shared launch(es): {}".format(
                ", ".join(unknown_shared)))

        raw_vehicles = message.get("vehicles")
        if not isinstance(raw_vehicles, list) or not raw_vehicles:
            raise ValueError("vehicles must be a non-empty list")
        if len(raw_vehicles) > self.max_vehicles:
            raise ValueError("vehicle count exceeds max_vehicles={}".format(self.max_vehicles))
        type_names = self.config.get("aircraft_types", {})
        sensor_types = self.config.get("sensor_types", {})
        vehicles = []
        names = set()
        uav_indices = set()
        for item in raw_vehicles:
            if not isinstance(item, dict):
                raise ValueError("each vehicle must be an object")
            name = item.get("name")
            type_name = item.get("type")
            if not isinstance(name, str) or not _NAME_RE.fullmatch(name):
                raise ValueError("invalid vehicle name: {}".format(name))
            if name in names:
                raise ValueError("duplicate vehicle {}".format(name))
            names.add(name)
            if type_name not in type_names:
                raise ValueError("unknown aircraft type for {}: {}".format(name, type_name))
            match = re.fullmatch(r"uav([1-9][0-9]*)", name)
            if not match:
                raise ValueError(
                    "vehicle name {} must be uav1, uav2, ... for video port mapping".format(
                        name))
            uav_index = int(match.group(1))
            if uav_index in uav_indices:
                raise ValueError("aircraft names map to duplicate video port {}".format(
                    32796 + uav_index))
            uav_indices.add(uav_index)
            if 32796 + uav_index > 65535:
                raise ValueError("video port for {} exceeds UDP port range".format(name))
            drone = "Drone{}".format(match.group(1)) if match else name
            vehicle_sensors = self._normalise_sensors(item.get("sensors", []))
            for sensor in vehicle_sensors:
                if sensor["type"] not in sensor_types:
                    raise ValueError("unknown sensor type {} on {}".format(
                        sensor["type"], name))
            launches = self._normalise_vehicle_launches(
                item.get("launches", []), type_name, vehicle_sensors, name)
            track_sensor = item.get("track_sensor")
            if track_sensor is not None:
                sensor_by_id = {sensor["id"]: sensor for sensor in vehicle_sensors}
                if not isinstance(track_sensor, str) or track_sensor not in sensor_by_id:
                    raise ValueError("track_sensor on {} must name one of its sensors".format(
                        name))
                if not sensor_by_id[track_sensor]["params"].get("image_topic"):
                    raise ValueError("track_sensor {} on {} needs image_topic".format(
                        track_sensor, name))
            params = item.get("params", {})
            if not isinstance(params, dict):
                raise ValueError("vehicle params for {} must be a mapping".format(name))
            vehicles.append({
                "name": name,
                "uav": name,
                "drone": drone,
                "uav_index": uav_index,
                "video_port": 32796 + uav_index,
                "type": type_name,
                "params": params,
                "sensors": vehicle_sensors,
                "launches": launches,
                "track_sensor": track_sensor,
            })
        return session_id, profile_name, vehicles, raw_shared

    def _command_argv(self, spec, context, label):
        argv = spec.get("argv")
        if not isinstance(argv, list) or not argv or any(not isinstance(arg, str) for arg in argv):
            raise ValueError("{} must define a non-empty argv list".format(label))
        expanded = [_expand_string(arg, context) for arg in argv]
        if not expanded[0]:
            raise ValueError("{} has an empty executable".format(label))
        return expanded

    @staticmethod
    def _ros_arg_value(value, label):
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (str, int, float)):
            value = str(value)
            if "\n" in value or "\r" in value:
                raise ValueError("{} cannot contain a newline".format(label))
            return value
        raise ValueError("{} must be a string, number or boolean".format(label))

    def _desired_processes(self, profile_name, profile, vehicles, shared_launches):
        context = self._fleet_context(profile_name, profile, vehicles, shared_launches)
        desired = {}

        def add(key, spec, local_context, label, selection=None, sensor=None):
            selection = selection or {"params": {}}
            argv = self._command_argv(spec, local_context, label)
            launch_params = _expand_tree(spec.get("default_params", {}), local_context)
            if not isinstance(launch_params, dict):
                raise ValueError("default_params for {} must be a mapping".format(label))
            if sensor is not None:
                for sensor_key, launch_key in spec.get("sensor_param_map", {}).items():
                    if sensor_key in sensor["params"]:
                        launch_params[launch_key] = sensor["params"][sensor_key]
            launch_params.update(_expand_tree(selection.get("params", {}), local_context))
            allowed_params = spec.get("allowed_params")
            if allowed_params is not None:
                unknown = sorted(set(launch_params) - set(allowed_params))
                if unknown:
                    raise ValueError("unsupported parameter(s) for {}: {}".format(
                        label, ", ".join(unknown)))
            prefix = str(spec.get("parameter_prefix", ""))
            for parameter, value in launch_params.items():
                if not isinstance(parameter, str) or not _ARG_NAME_RE.fullmatch(parameter):
                    raise ValueError("invalid ROS argument name for {}: {}".format(
                        label, parameter))
                argv.append("{}{}:={}".format(
                    prefix, parameter, self._ros_arg_value(value, label)))
            signature = hashlib.sha256(json.dumps(argv, ensure_ascii=False).encode(
                "utf-8")).hexdigest()
            desired[key] = {"argv": argv, "signature": signature, "label": label}

        for launch_id in shared_launches:
            spec = self.config["shared_launches"][launch_id]
            add("shared:{}/shared".format(launch_id), spec, context, launch_id)

        for vehicle in vehicles:
            vehicle_context = copy.deepcopy(context)
            vehicle_context.update(vehicle["params"])
            vehicle_context.update({
                "name": vehicle["name"], "uav": vehicle["name"],
                "uav_id": vehicle["name"], "drone": vehicle["drone"],
                "type": vehicle["type"], "uav_index": vehicle["uav_index"],
                "video_port": vehicle["video_port"],
                "video_topic": "/{}/track/phase_annotated".format(vehicle["name"]),
            })
            profile_files = profile.get("files", {})
            tf_path = profile_files.get("tf_config_by_type", {}).get(vehicle["type"])
            if tf_path:
                vehicle_context["tf_config"] = self._profile_path(tf_path)
            controller_path = profile_files.get("controller_config_by_type", {}).get(
                vehicle["type"])
            if controller_path:
                vehicle_context["controller_config"] = self._profile_path(controller_path)

            sensor_by_id = {sensor["id"]: sensor for sensor in vehicle["sensors"]}
            sensor_contexts = {}
            for sensor_id, sensor in sensor_by_id.items():
                sensor_context = copy.deepcopy(vehicle_context)
                sensor_context.update(sensor["params"])
                sensor_context.setdefault("image_topic", "")
                sensor_context.setdefault("camera_info_topic", "")
                sensor_context.setdefault("point_cloud_topic", "")
                sensor_context.setdefault("laser_range_topic", "")
                sensor_context.update({
                    "sensor": sensor_id,
                    "sensor_instance": sensor_id,
                    "sensor_id": sensor["params"].get("sensor_id", sensor_id),
                    "sensor_type": sensor["type"],
                })
                sensor_contexts[sensor_id] = sensor_context

            sar_contexts = {}
            for selection in vehicle["launches"]:
                if selection["id"] != "sar_yolo_detector":
                    continue
                sensor = sensor_by_id[selection["sensor"]]
                sensor_context = copy.deepcopy(sensor_contexts[sensor["id"]])
                sensor_context.update(selection["params"])
                instance_name = str(sensor_context.get("instance_name", sensor["id"]))
                topic_root = "sar_yolo_detector/{}".format(instance_name)
                sensor_context.setdefault("instance_name", instance_name)
                sensor_context.setdefault("input_image_topic",
                                          sensor_context.get("image_topic", ""))
                sensor_context.setdefault("camera_info_topic", "")
                sensor_context.setdefault(
                    "tracked_detections_topic", topic_root + "/tracked_detections")
                sensor_context.setdefault(
                    "fusion_detections_topic",
                    sensor_context["tracked_detections_topic"] + "/fusion")
                sensor_context.setdefault(
                    "fusion_observations_topic",
                    sensor_context["tracked_detections_topic"] + "/observations")
                sensor_context.setdefault(
                    "selected_detections_topic",
                    topic_root + "/selected_tracked_detections")
                sensor_context.setdefault("select_pixel_topic", topic_root + "/select_pixel")
                sensor_context.setdefault("auto_select_service", topic_root + "/auto_select")
                sensor_context.setdefault(
                    "auto_select_status_topic", topic_root + "/auto_select_status")
                sensor_context.setdefault("state_topic", topic_root + "/state")
                sensor_context.setdefault("annotated_image_topic", topic_root + "/annotated")
                sensor_context.setdefault("gimbal_detection_topic",
                                          sensor_context["selected_detections_topic"])
                sensor_context.setdefault("bridge_input_topic",
                                          sensor_context["selected_detections_topic"])
                sensor_context.setdefault("bridge_input_type", "tracked")
                sensor_context.setdefault("output_detections_topic", "track/selected_detections")
                sensor_context.setdefault("detect_output_detections_topic",
                                          "detect/{}/input/detections_2d".format(sensor["id"]))
                sensor_context["sar_vehicle_config"] = self._generate_sar_overlay(
                    sensor_context)
                sensor_context.update({
                    "config": sensor_context.get("sar_base_config", ""),
                    "vehicle_config": sensor_context["sar_vehicle_config"],
                    "fusion_config": sensor_context.get("sar_fusion_config", ""),
                    "model_path": sensor_context.get("smart_model_path", ""),
                    "model_sha256": sensor_context.get("smart_model_sha256", ""),
                    "python_executable": sensor_context.get("sar_python", ""),
                })
                sar_contexts[sensor["id"]] = sensor_context

            if vehicle["track_sensor"]:
                selected_track_sensor = vehicle["track_sensor"]
                selected_sar = sar_contexts.get(selected_track_sensor)
                fallback_image_topic = sensor_by_id[selected_track_sensor]["params"].get(
                    "image_topic")
            else:
                selected_sar = next(iter(sar_contexts.values()), None)
                fallback_image_topic = next((sensor["params"].get("image_topic")
                                             for sensor in vehicle["sensors"]
                                             if sensor["params"].get("image_topic")), None)
            track_template = context.get("track_config_template")
            if not track_template:
                raise ValueError("selected profile has no track_config_template")
            vehicle_context["track_config"] = self._generate_track_config(
                track_template, vehicle["name"], selected_sar, fallback_image_topic)

            required_launches = self.config.get("required_vehicle_launches", [])
            selections = [{"id": launch_id, "sensor": None, "params": {}}
                          for launch_id in required_launches]
            selections.extend(vehicle["launches"])

            for selection in selections:
                launch_id = selection["id"]
                sensor_id = selection["sensor"]
                sensor = sensor_by_id.get(sensor_id) if sensor_id else None
                spec = self._vehicle_launch_spec(vehicle["type"], launch_id)
                local_context = (copy.deepcopy(sar_contexts[sensor_id])
                                 if launch_id == "sar_yolo_detector" else
                                 copy.deepcopy(sensor_contexts[sensor_id])
                                 if sensor_id else copy.deepcopy(vehicle_context))
                if launch_id == "detect" and sensor and sensor["type"] == "lidar":
                    local_context["detect_config"] = local_context.get(
                        "lidar_detect_config", local_context.get("detect_config", ""))
                key = "vehicle:{}/launch:{}".format(vehicle["name"], launch_id)
                if sensor_id:
                    key += ":sensor:{}".format(sensor_id)
                add(key, spec, local_context,
                    "{}:{}{}".format(vehicle["name"], launch_id,
                                      ":" + sensor_id if sensor_id else ""),
                    selection, sensor)
        return desired

    def _wrapped_argv(self, argv):
        setup = " && ".join("source {}".format(shlex.quote(path))
                            for path in self.setup_scripts)
        command = (setup + " && " if setup else "") + 'exec "$@"'
        return ["bash", "-lc", command, "xd-uav-managed-launch"] + argv

    def _start_process(self, key, spec):
        log_dir = self.runtime_dir / "logs"
        log_dir.mkdir(mode=0o700, exist_ok=True)
        log_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", key)[:180] + ".log"
        log_path = log_dir / log_name
        log_stream = open(log_path, "ab", buffering=0)
        argv = self._wrapped_argv(spec["argv"])
        process = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            close_fds=True,
            start_new_session=True,
            env=os.environ.copy(),
        )
        log_stream.close()
        pgid, start_time = _proc_info(process.pid)
        if pgid is None or start_time is None:
            process.terminate()
            raise RuntimeError("could not record process identity for {}".format(key))
        record = {
            "pid": process.pid,
            "pgid": pgid,
            "start_time": start_time,
            "signature": spec["signature"],
            "command": spec["argv"],
            "log_path": str(log_path),
            "label": spec["label"],
        }
        self.active[key] = {"record": record, "process": process}
        self._write_registry()
        self._log("started {} pid={} log={}".format(
            spec["label"], process.pid, log_path))

    def _stop_processes(self, keys):
        stopping = []
        for key in keys:
            managed = self.active.pop(key, None)
            if managed:
                stopping.append((key, managed))
        if not stopping:
            return

        self._signal_records([managed["record"] for _, managed in stopping])
        for key, managed in stopping:
            process = managed.get("process")
            if process is not None:
                process.poll()
            record = managed["record"]
            self._log("stopped {} pid={}".format(record.get("label", key), record["pid"]))

    def _reconcile(self, desired):
        stale_keys = []
        for key, managed in list(self.active.items()):
            wanted = desired.get(key)
            process = managed.get("process")
            alive = process is None or process.poll() is None
            same = wanted and wanted["signature"] == managed["record"].get("signature")
            if not same or not alive:
                stale_keys.append(key)
        self._stop_processes(stale_keys)
        for key, spec in desired.items():
            if key not in self.active:
                self._start_process(key, spec)
        self._write_registry()

    def _stop_all(self):
        self._stop_processes(list(self.active))
        self.active.clear()
        old_generation = self.generation_dir
        self.generation_dir = None
        self.session_id = None
        self._write_registry()
        if old_generation and old_generation.exists():
            shutil.rmtree(old_generation, ignore_errors=True)

    def _check_auth(self, message):
        if self.require_auth and not self.auth_token:
            raise ValueError("authentication is required but no server token is configured")
        if self.auth_token and not hmac.compare_digest(
                str(message.get("token", "")), self.auth_token):
            raise ValueError("authentication failed")

    def _handle_message(self, message):
        self._check_auth(message)
        action = message.get("action")
        if action in ("start", "update", "state"):
            session_id, profile_name, vehicles, shared_launches = self._validate_snapshot(message)
            if self.session_id and self.session_id != session_id:
                self._log("session changed from {} to {}; stopping old processes".format(
                    self.session_id, session_id))
                self._stop_all()
            self.session_id = session_id
            profile = self.config["profiles"][profile_name]
            desired = self._desired_processes(
                profile_name, profile, vehicles, shared_launches)
            self._reconcile(desired)
            return {"ok": True, "action": action, "session_id": session_id,
                    "profile": profile_name, "active": self._status()}
        if action == "heartbeat":
            return {"ok": True, "action": action, "session_id": self.session_id,
                    "active_count": len(self.active)}
        if action == "status":
            return {"ok": True, "session_id": self.session_id,
                    "active": self._status()}
        if action == "stop":
            self._stop_all()
            return {"ok": True, "action": "stop", "active": []}
        raise ValueError("action must be start, update, heartbeat, status or stop")

    def _status(self):
        result = []
        for key, managed in self.active.items():
            record = managed["record"]
            process = managed.get("process")
            result.append({
                "key": key,
                "pid": record["pid"],
                "label": record.get("label"),
                "running": process is None or process.poll() is None,
                "log_path": record.get("log_path"),
            })
        return result

    def _send(self, connection, value):
        raw = (json.dumps(value, ensure_ascii=False) + "\n").encode("utf-8")
        connection.sendall(raw)

    def _serve_connection(self, connection, address):
        self._log("connected host {}:{}".format(*address))
        buffer = bytearray()
        last_message = time.monotonic()
        connection.settimeout(1.0)
        should_close = False
        try:
            while not self.shutdown_event.is_set() and not should_close:
                if time.monotonic() - last_message > self.timeout_sec:
                    self._log("host heartbeat timeout; stopping active processes")
                    break
                try:
                    chunk = connection.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    self._log("host disconnected; stopping active processes")
                    break
                buffer.extend(chunk)
                if len(buffer) > self.max_message_bytes:
                    self._send(connection, {"ok": False, "error": "message too large"})
                    break
                while b"\n" in buffer:
                    raw_line, _, remaining = buffer.partition(b"\n")
                    buffer = bytearray(remaining)
                    if not raw_line.strip():
                        continue
                    last_message = time.monotonic()
                    try:
                        message = json.loads(raw_line.decode("utf-8"))
                        if not isinstance(message, dict):
                            raise ValueError("each line must be a JSON object")
                        response = self._handle_message(message)
                        self._send(connection, response)
                        if message.get("action") == "stop":
                            should_close = True
                            break
                    except Exception as error:
                        self._send(connection, {"ok": False, "error": str(error)})
                        self._log("request error from {}:{}: {}".format(
                            address[0], address[1], error))
        except (ConnectionError, OSError) as error:
            self._log("connection ended: {}".format(error))
        finally:
            try:
                connection.close()
            except OSError:
                pass
            self._stop_all()

    def serve_forever(self):
        network = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = network
        network.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        network.bind((self.host, self.port))
        network.listen(4)
        network.settimeout(1.0)
        self._log("listening on {}:{} (heartbeat timeout {}s)".format(
            self.host, self.port, self.timeout_sec))
        try:
            while not self.shutdown_event.is_set():
                try:
                    connection, address = network.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self.shutdown_event.is_set():
                        break
                    raise
                self._serve_connection(connection, address)
        finally:
            try:
                network.close()
            except OSError:
                pass
            self._stop_all()

    def request_shutdown(self, _signum=None, _frame=None):
        self.shutdown_event.set()
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default_config = Path(__file__).with_name("manager.yaml")
    parser.add_argument("--config", default=str(default_config),
                        help="manager YAML configuration")
    args = parser.parse_args()
    try:
        manager = ProcessManager(args.config)
    except Exception as error:
        print("[start-manager] startup error: {}".format(error),
              file=sys.stderr, flush=True)
        return 1
    signal.signal(signal.SIGINT, manager.request_shutdown)
    signal.signal(signal.SIGTERM, manager.request_shutdown)
    try:
        manager.serve_forever()
    except Exception as error:
        print("[start-manager] fatal: {}".format(error), file=sys.stderr, flush=True)
        manager._stop_all()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
