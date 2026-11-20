#!/usr/bin/env python3
"""TCP-controlled launcher for the ROS processes used by the UAV stack."""

import argparse
import atexit
import concurrent.futures
import copy
import fcntl
import hashlib
import hmac
import http.client
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
import xmlrpc.client
from urllib.parse import urlparse

import yaml


_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_ARG_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_PLACEHOLDER_RE = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")
_TERM_GRACE_SEC = 2.0
_KILL_GRACE_SEC = 1.0
_ROS_CALL_TIMEOUT_SEC = 0.5


class _TimeoutTransport(xmlrpc.client.Transport):
    """XML-RPC transport with a bounded connect/read timeout."""

    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        if self._connection and self._connection[0] == host:
            return self._connection[1]
        connection = http.client.HTTPConnection(host, timeout=self.timeout)
        self._connection = (host, connection)
        return connection


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


def _process_group_exists(pgid):
    try:
        os.killpg(int(pgid), 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _proc_snapshot():
    """Return the live process tree fields needed for managed cleanup."""
    snapshot = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "stat").read_text(encoding="utf-8")
            fields = raw[raw.rfind(")") + 2:].split()
            snapshot[int(entry.name)] = {
                "ppid": int(fields[1]),
                "pgid": int(fields[2]),
                "start_time": fields[19],
            }
        except (OSError, ValueError, IndexError):
            continue
    return snapshot


def _descendants(snapshot, root_pid):
    children = {}
    for pid, details in snapshot.items():
        children.setdefault(details["ppid"], []).append(pid)

    result = []
    pending = list(children.get(root_pid, []))
    seen = set()
    while pending:
        pid = pending.pop()
        if pid in seen:
            continue
        seen.add(pid)
        result.append(pid)
        pending.extend(children.get(pid, []))
    return result


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
        self.node_startup_timeout_sec = max(
            1.0, float(self.config.get("node_startup_timeout_sec", 15.0)))
        self.node_status_check_interval_sec = max(
            1.0, float(self.config.get("node_status_check_interval_sec", 10.0)))
        self.node_reuse_check_interval_sec = max(
            1.0, float(self.config.get("node_reuse_check_interval_sec", 10.0)))
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
        self.reused_launches = {}
        self.last_node_status_check = 0.0
        self.last_client_activity = time.monotonic()
        self.session_id = None
        self.generation_dir = None
        self.shutdown_event = threading.Event()
        self.server_socket = None
        self.ros_cleanup_done = False
        self._recover_previous_processes()

    def _log(self, message):
        print("[start-manager] " + message, flush=True)

    def _cleanup_ros_master(self):
        """Run global stale-registration cleanup once when a session ends."""
        if self.ros_cleanup_done:
            return True
        rosnode = shutil.which("rosnode")
        if not rosnode:
            self._log("rosnode cleanup unavailable; source the ROS environment first")
            return False
        try:
            result = subprocess.run(
                [rosnode, "cleanup"],
                input="y\n",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=180.0,
                env=os.environ.copy(),
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            self._log("rosnode cleanup timed out")
            return False
        for line in (result.stdout or "").splitlines():
            self._log("rosnode cleanup: " + line)
        if result.returncode != 0:
            self._log("rosnode cleanup failed with exit code {}".format(
                result.returncode))
            return False
        self.ros_cleanup_done = True
        return True

    @staticmethod
    def _xmlrpc_proxy(uri):
        return xmlrpc.client.ServerProxy(
            uri, allow_none=True,
            transport=_TimeoutTransport(_ROS_CALL_TIMEOUT_SEC))

    def _registered_ros_nodes(self):
        master_uri = os.environ.get("ROS_MASTER_URI", "").strip()
        if not master_uri:
            raise RuntimeError("ROS_MASTER_URI is not set")
        with self._xmlrpc_proxy(master_uri) as master:
            code, message, state = master.getSystemState("/xd_uav_start_manager")
        if code != 1:
            raise RuntimeError("ROS master getSystemState failed: {}".format(message))
        nodes = set()
        for entries in state:
            for _resource, node_names in entries:
                nodes.update(node_names)
        return nodes

    def _ros_node_owner(self, node_name):
        master_uri = os.environ.get("ROS_MASTER_URI", "").strip()
        with self._xmlrpc_proxy(master_uri) as master:
            code, message, node_uri = master.lookupNode(
                "/xd_uav_start_manager", node_name)
        if code != 1:
            raise RuntimeError("ROS master lookupNode failed: {}".format(message))
        with self._xmlrpc_proxy(node_uri) as node:
            code, message, pid = node.getPid("/xd_uav_start_manager")
        if code != 1:
            raise RuntimeError("node getPid failed: {}".format(message))
        return node_uri, int(pid)

    @staticmethod
    def _node_uri_is_local(uri):
        try:
            host = urlparse(uri).hostname
        except (TypeError, ValueError):
            return False
        if not host:
            return False
        local_names = {
            "localhost", socket.gethostname(), socket.getfqdn(),
            os.environ.get("ROS_IP", ""), os.environ.get("ROS_HOSTNAME", ""),
        }
        local_names.discard("")
        if host in local_names:
            return True
        try:
            host_addresses = {
                item[4][0] for item in socket.getaddrinfo(host, None)
            }
            local_addresses = {"127.0.0.1", "::1"}
            for name in local_names:
                try:
                    local_addresses.update(
                        item[4][0] for item in socket.getaddrinfo(name, None))
                except OSError:
                    continue
            return bool(host_addresses & local_addresses)
        except OSError:
            return False

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

    @staticmethod
    def _validate_expected_node_names(desired):
        owners = {}
        for key, spec in desired.items():
            for node_name in spec.get("expected_nodes", []):
                previous = owners.get(node_name)
                if previous is not None and previous != key:
                    raise ValueError(
                        "expected ROS node {} is claimed by both {} and {}".format(
                            node_name, previous, key))
                owners[node_name] = key

    def _signal_records(self, records):
        """Stop launch groups and ROS descendants; return groups still alive."""
        snapshot = _proc_snapshot()
        groups_by_record = {}
        group_members = {}
        for record in records:
            try:
                pid = int(record.get("pid", -1))
                pgid = int(record.get("pgid", -1))
                expected_start = str(record.get("start_time", ""))
            except (AttributeError, TypeError, ValueError):
                self._log("skipping malformed process record")
                continue
            if pid <= 1 or pgid != pid:
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
                    groups_by_record[pid] = {pgid}
                    group_members.setdefault(pgid, {})
                    continue

            owned_groups = {pgid}
            group_members.setdefault(pgid, {})
            if actual_start is not None:
                group_members[pgid][pid] = expected_start
            for child_pid in _descendants(snapshot, pid):
                child = snapshot[child_pid]
                child_pgid = child["pgid"]
                if child_pgid <= 1:
                    continue
                expected_child_start = child["start_time"]
                actual_child_pgid, actual_child_start = _proc_info(child_pid)
                if (actual_child_pgid != child_pgid or
                        actual_child_start != expected_child_start):
                    continue
                owned_groups.add(child_pgid)
                group_members.setdefault(child_pgid, {})[child_pid] = expected_child_start
            groups_by_record[pid] = owned_groups

        def group_is_owned_and_alive(pgid):
            members = group_members.get(pgid, {})
            if not members:
                return _process_group_exists(pgid)
            for member_pid, expected_start in members.items():
                actual_pgid, actual_start = _proc_info(member_pid)
                if actual_pgid == pgid and actual_start == expected_start:
                    return True
            return False

        # Ask roslaunch and its nodes to shut down cleanly first. ROS nodes can
        # have their own process groups, so signal each owned group explicitly.
        for pgid in group_members:
            if group_is_owned_and_alive(pgid):
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    self._log("permission denied stopping process group {}".format(pgid))

        deadline = time.monotonic() + _TERM_GRACE_SEC
        while time.monotonic() < deadline:
            if not any(group_is_owned_and_alive(pgid) for pgid in group_members):
                break
            time.sleep(0.05)

        # Escalate only groups that survived the graceful shutdown window.
        for pgid in group_members:
            if group_is_owned_and_alive(pgid):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except PermissionError:
                    self._log("permission denied killing process group {}".format(pgid))

        deadline = time.monotonic() + _KILL_GRACE_SEC
        while time.monotonic() < deadline:
            if not any(group_is_owned_and_alive(pgid) for pgid in group_members):
                break
            time.sleep(0.05)

        remaining = {}
        for pid, groups in groups_by_record.items():
            survivors = {pgid for pgid in groups if group_is_owned_and_alive(pgid)}
            if survivors:
                remaining[pid] = survivors
                self._log("process {} still has live group(s): {}".format(
                    pid, ", ".join(str(group) for group in sorted(survivors))))
        return remaining

    def _recover_previous_processes(self):
        registry = self._load_registry()
        records = registry.get("processes", {})
        if isinstance(records, dict) and records:
            self._log("stopping {} process group(s) left by the previous manager".format(
                len(records)))
            remaining = self._signal_records(list(records.values()))
            if remaining:
                raise RuntimeError(
                    "could not stop all processes left by the previous manager; "
                    "records were retained for recovery"
                )
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

        for type_name, path in files.get("tf_config_by_type", {}).items():
            context["tf_config_" + str(type_name)] = self._profile_path(path)
        for type_name, path in files.get("controller_config_by_type", {}).items():
            context["controller_config_" + str(type_name)] = self._profile_path(path)

        for key, value in files.items():
            if (key.endswith("_config") or key.endswith("_config_template")) and isinstance(value, str):
                context[key] = self._profile_path(value)
        return context

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
            if spec.get("ignore_selection_params", False):
                params = {}
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
        required_shared = self.config.get("required_shared_launches", [])
        if not isinstance(required_shared, list) or any(
                not isinstance(value, str) or value not in shared_specs
                for value in required_shared):
            raise ValueError("required_shared_launches has an unknown launch id")
        unknown_shared = sorted(set(raw_shared) - set(shared_specs))
        if unknown_shared:
            raise ValueError("unknown shared launch(es): {}".format(
                ", ".join(unknown_shared)))
        effective_shared = []
        for launch_id in required_shared + raw_shared:
            if launch_id not in effective_shared:
                effective_shared.append(launch_id)

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
            })
        return session_id, profile_name, vehicles, effective_shared

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
            selection_params = (
                {} if spec.get("ignore_selection_params", False) else
                _expand_tree(selection.get("params", {}), local_context))
            selection_param_map = spec.get("selection_param_map", {})
            for parameter, value in selection_params.items():
                launch_params[selection_param_map.get(parameter, parameter)] = value
            forced_params = _expand_tree(spec.get("forced_params", {}), local_context)
            if not isinstance(forced_params, dict):
                raise ValueError("forced_params for {} must be a mapping".format(label))
            launch_params.update(forced_params)
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
            expected_nodes = _expand_tree(spec.get("expected_nodes", []), local_context)
            if (not isinstance(expected_nodes, list) or
                    any(not isinstance(node, str) or not node.startswith("/")
                        for node in expected_nodes)):
                raise ValueError("expected_nodes for {} must be a list of absolute ROS names".format(
                    label))
            expected_nodes = list(dict.fromkeys(expected_nodes))
            signature_payload = ({"argv": argv, "expected_nodes": expected_nodes}
                                 if expected_nodes else argv)
            signature = hashlib.sha256(json.dumps(
                signature_payload, ensure_ascii=False).encode(
                "utf-8")).hexdigest()
            desired[key] = {
                "argv": argv, "signature": signature, "label": label,
                "expected_nodes": expected_nodes,
            }

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
                "video_topic": "/{}/sar_yolo_detector/smart/annotated".format(vehicle["name"]),
            })
            profile_files = profile.get("files", {})
            tf_path = profile_files.get("tf_config_by_type", {}).get(vehicle["type"])
            if tf_path:
                vehicle_context["tf_config"] = self._profile_path(tf_path)
            controller_path = profile_files.get("controller_config_by_type", {}).get(
                vehicle["type"])
            if controller_path:
                vehicle_context["controller_config"] = self._profile_path(controller_path)

            for sensor in vehicle["sensors"]:
                camera_config = {
                    "front_gimbal_camera": ("FrontCamera", "front_camera"),
                    "down_camera": ("DownCamera", "down_camera"),
                }.get(sensor["type"])
                if not camera_config:
                    continue
                camera_name, camera_topic_prefix = camera_config
                sensor_params = sensor["params"]
                sensor_params.setdefault("sim_image_topic",
                                         sensor_params.get("image_topic", ""))
                sensor_params.setdefault("sim_camera_info_topic",
                                         sensor_params.get("camera_info_topic", ""))
                sensor_params.setdefault(
                    "camera_stream_name", "{}/{}".format(vehicle["drone"], camera_name))
                sensor_params["image_topic"] = (
                    "/{}/{}/image_raw".format(vehicle["name"], camera_topic_prefix))
                sensor_params["camera_info_topic"] = (
                    "/{}/{}/camera_info".format(vehicle["name"], camera_topic_prefix))

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
                # The SAR launch command consumes these camera inputs. The
                # selected/annotated topics below keep the other auto-starts
                # aligned with the launch defaults; model options also use
                # xd_smart_tracker_fusion_integration.launch defaults.
                topic_root = "sar_yolo_detector/smart"
                sensor_context.setdefault("input_image_topic",
                                          sensor_context.get("image_topic", ""))
                sensor_context.setdefault("camera_info_topic", "")
                sensor_context["selected_detections_topic"] = (
                    topic_root + "/selected_tracked_detections")
                sensor_context["annotated_image_topic"] = topic_root + "/annotated"
                sar_contexts[sensor["id"]] = sensor_context

            for sensor_id, sensor in sensor_by_id.items():
                sar_context = sar_contexts.get(sensor_id)
                detection_topic = (
                    sar_context.get("selected_detections_topic") if sar_context else None
                ) or sensor["params"].get("detection_topic")
                if not detection_topic:
                    detection_topic = (
                        "sar_yolo_detector/smart/selected_tracked_detections")
                if not str(detection_topic).startswith("/"):
                    detection_topic = "/{}/{}".format(
                        vehicle["name"], str(detection_topic).lstrip("/"))
                sensor_contexts[sensor_id]["automatic_detection_topic"] = detection_topic

            required_launches = self.config.get("required_vehicle_launches", [])
            selections = [{"id": launch_id, "sensor": None, "params": {}}
                          for launch_id in required_launches]
            automatic_launches = []
            for sensor in vehicle["sensors"]:
                configured = self.config.get("automatic_sensor_launches", {}).get(
                    sensor["type"], [])
                for item in configured:
                    if isinstance(item, str):
                        launch_id, params = item, {}
                    else:
                        launch_id = item["id"]
                        params = copy.deepcopy(item.get("params", {}))
                    automatic_launches.append({
                        "id": launch_id, "sensor": sensor["id"], "params": params})

            explicit_by_identity = {
                (item["id"], item["sensor"]): item for item in vehicle["launches"]}
            automatic_identities = set()
            for automatic in automatic_launches:
                identity = (automatic["id"], automatic["sensor"])
                automatic_identities.add(identity)
                selections.append(explicit_by_identity.get(identity, automatic))
            selections.extend(
                item for item in vehicle["launches"]
                if (item["id"], item["sensor"]) not in automatic_identities)

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
                # Track and H.264 consume the UAV-wide annotated topic, so a
                # vehicle with both cameras still owns only one process each.
                key = "vehicle:{}/launch:{}".format(vehicle["name"], launch_id)
                if sensor_id and not spec.get("once_per_vehicle", False):
                    key += ":sensor:{}".format(sensor_id)
                add(key, spec, local_context,
                    "{}:{}{}".format(vehicle["name"], launch_id,
                                      ":" + sensor_id if sensor_id else ""),
                    selection, sensor)
        return desired

    def _wrapped_argv(self, argv):
        setup = " && ".join("source {}".format(shlex.quote(path))
                            for path in self.setup_scripts)
        command = (setup + " && " if setup else "") + (
            "unset ROS_NAMESPACE ROS_HOSTNAME && exec \"$@\"")
        return ["bash", "-lc", command, "xd-uav-managed-launch"] + argv

    def _start_process(self, key, spec):
        log_dir = self.runtime_dir / "logs"
        log_dir.mkdir(mode=0o700, exist_ok=True)
        log_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", key)[:180] + ".log"
        log_path = log_dir / log_name
        if log_path.exists():
            previous = log_path.read_bytes()
            try:
                previous.decode("utf-8")
            except UnicodeDecodeError:
                repaired_path = log_path.with_suffix(".log.repair")
                repaired_path.write_bytes(
                    previous.decode("utf-8", errors="replace").encode("utf-8"))
                os.replace(repaired_path, log_path)
        log_stream = open(log_path, "ab", buffering=0)
        log_stream.write(("\n\n===== launch started {} =====\n".format(
            time.strftime("%Y-%m-%d %H:%M:%S"))).encode("utf-8"))
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
            "expected_nodes": list(spec.get("expected_nodes", [])),
        }
        expected_nodes = list(spec.get("expected_nodes", []))
        self.active[key] = {
            "record": record,
            "process": process,
            "expected_nodes": expected_nodes,
            "started_monotonic": time.monotonic(),
            "health": "starting" if expected_nodes else "running",
            "health_reason": "waiting for ROS node registration" if expected_nodes else "",
            "node_states": {node: "waiting" for node in expected_nodes},
        }
        self._write_registry()
        self._log("started {} pid={} log={}".format(
            spec["label"], process.pid, log_path))

    def _stop_processes(self, keys):
        stopping = []
        for key in keys:
            managed = self.active.get(key)
            if managed:
                stopping.append((key, managed))
        if not stopping:
            return

        remaining_by_pid = self._signal_records(
            [managed["record"] for _, managed in stopping])
        for key, managed in stopping:
            process = managed.get("process")
            if process is not None:
                try:
                    process.wait(timeout=_KILL_GRACE_SEC)
                except subprocess.TimeoutExpired:
                    pass
            record = managed["record"]
            pid = int(record["pid"])
            remaining_groups = remaining_by_pid.get(pid, set())
            remaining_groups = {
                pgid for pgid in remaining_groups if _process_group_exists(pgid)
            }
            if ((process is not None and process.poll() is None) or remaining_groups):
                self._log("cleanup incomplete for {}; keeping its process record".format(
                    record.get("label", key)))
                continue
            self.active.pop(key, None)
            self._log("stopped {} pid={}".format(record.get("label", key), record["pid"]))

    def _find_reusable_launches(self, start_specs):
        """Reuse complete, responsive ROS launch groups already on the master."""
        expected_by_key = {
            key: list(spec.get("expected_nodes", []))
            for key, spec in start_specs if spec.get("expected_nodes")
        }
        expected_nodes = sorted({
            node_name for nodes in expected_by_key.values() for node_name in nodes
        })
        if not expected_nodes:
            return {}

        try:
            registered = self._registered_ros_nodes()
        except Exception as error:
            self._log("ROS master unavailable during reuse check; starting configured "
                      "launches normally: {}".format(error))
            return {}

        registered_expected = sorted(set(expected_nodes) & registered)
        if not registered_expected:
            return {}

        owners = {}
        if registered_expected:
            with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(16, len(registered_expected))) as executor:
                futures = {
                    executor.submit(self._ros_node_owner, node_name): node_name
                    for node_name in registered_expected
                }
                for future, node_name in futures.items():
                    try:
                        owners[node_name] = future.result()
                    except Exception as error:
                        owners[node_name] = error

        stale = [node_name for node_name, owner in owners.items()
                 if isinstance(owner, Exception)]
        if stale:
            self._log("registered expected node(s) are unreachable; cleaning stale ROS "
                      "registrations once: {}".format(", ".join(stale)))
            self.ros_cleanup_done = False
            cleanup_ok = self._cleanup_ros_master()
            # A cleanup during an active session must not suppress cleanup at
            # the eventual session end.
            self.ros_cleanup_done = False
            if cleanup_ok:
                try:
                    registered = self._registered_ros_nodes()
                    registered_expected = sorted(set(expected_nodes) & registered)
                except Exception as error:
                    self._log("could not recheck ROS registrations after cleanup; "
                              "starting configured launches: {}".format(error))
                    return {}
                if registered_expected:
                    with concurrent.futures.ThreadPoolExecutor(
                            max_workers=min(16, len(registered_expected))) as executor:
                        futures = {
                            executor.submit(self._ros_node_owner, node_name): node_name
                            for node_name in registered_expected
                        }
                        owners = {}
                        for future, node_name in futures.items():
                            try:
                                owners[node_name] = future.result()
                            except Exception as error:
                                owners[node_name] = error

        reusable = {}
        now = time.monotonic()
        for key, nodes in expected_by_key.items():
            if not nodes or not set(nodes).issubset(registered):
                continue
            node_owners = {node_name: owners.get(node_name) for node_name in nodes}
            if any(not isinstance(owner, tuple) for owner in node_owners.values()):
                continue
            spec = next(spec for candidate_key, spec in start_specs
                        if candidate_key == key)
            reusable[key] = {
                "signature": spec["signature"],
                "label": spec["label"],
                "expected_nodes": list(nodes),
                "owners": node_owners,
                "next_check": now + self.node_reuse_check_interval_sec,
            }
            self._log("reusing already-running {} with registered nodes {}".format(
                spec["label"], ", ".join(nodes)))
        return reusable

    def _reconcile(self, desired):
        started = []
        stale_keys = []
        for key, managed in list(self.active.items()):
            wanted = desired.get(key)
            process = managed.get("process")
            alive = process is None or process.poll() is None
            same = wanted and wanted["signature"] == managed["record"].get("signature")
            if not same or not alive:
                stale_keys.append(key)
        self._stop_processes(stale_keys)
        still_running = [key for key in stale_keys if key in self.active]
        if still_running:
            raise RuntimeError(
                "could not stop replaced managed process(es); refusing to start a "
                "second copy: {}".format(", ".join(still_running)))
        start_specs = [(key, spec) for key, spec in desired.items()
                       if key not in self.active]
        self.reused_launches = {
            key: reused for key, reused in self.reused_launches.items()
            if key in desired and key not in self.active and
            desired[key]["signature"] == reused.get("signature")
        }
        now = time.monotonic()
        cached_reuse = {
            key: self.reused_launches[key] for key, spec in start_specs
            if key in self.reused_launches and
            now < self.reused_launches[key].get("next_check", 0.0) and
            self.reused_launches[key].get("signature") == spec["signature"]
        }
        self.reused_launches = dict(cached_reuse)
        reuse_candidates = [(key, spec) for key, spec in start_specs
                            if key not in cached_reuse]
        newly_reused = self._find_reusable_launches(reuse_candidates)
        self.reused_launches.update(newly_reused)
        for key, spec in start_specs:
            if key in self.reused_launches:
                continue
            self._start_process(key, spec)
            started.append(key)
        self._write_registry()
        return started, bool(stale_keys or started)

    def _refresh_node_status(self, force=False):
        now = time.monotonic()
        if (not force and now - self.last_node_status_check <
                self.node_status_check_interval_sec):
            return
        self.last_node_status_check = now
        expected = sorted({
            node for managed in self.active.values()
            for node in managed.get("expected_nodes", [])
        })
        if not expected:
            for managed in self.active.values():
                managed["health"] = (
                    "running" if managed["process"].poll() is None else "launch_exited")
            return

        try:
            registered = self._registered_ros_nodes()
        except Exception as error:
            for key, managed in self.active.items():
                if managed.get("expected_nodes"):
                    self._set_node_health(
                        key, managed, "ros_master_unavailable", str(error), {})
            return

        nodes_to_probe = sorted(set(expected) & registered)
        owners = {}
        if nodes_to_probe:
            with concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(16, len(nodes_to_probe))) as executor:
                futures = {
                    executor.submit(self._ros_node_owner, node): node
                    for node in nodes_to_probe
                }
                for future, node in futures.items():
                    try:
                        owners[node] = future.result()
                    except Exception as error:
                        owners[node] = error

        process_snapshot = _proc_snapshot()
        for key, managed in list(self.active.items()):
            process = managed["process"]
            if process.poll() is not None:
                self._set_node_health(
                    key, managed, "launch_exited",
                    "roslaunch exited with code {}".format(process.returncode), {})
                continue
            expected_nodes = managed.get("expected_nodes", [])
            if not expected_nodes:
                self._set_node_health(key, managed, "running", "", {})
                continue

            root_pid = int(managed["record"]["pid"])
            managed_pids = {root_pid, *_descendants(process_snapshot, root_pid)}
            node_states = {}
            replaced = []
            unverified = []
            for node_name in expected_nodes:
                if node_name not in registered:
                    node_states[node_name] = "missing"
                    continue
                owner = owners.get(node_name)
                if isinstance(owner, Exception):
                    node_states[node_name] = "unverified"
                    unverified.append("{} ({})".format(node_name, owner))
                    continue
                node_uri, node_pid = owner
                if (not self._node_uri_is_local(node_uri) or
                        node_pid not in managed_pids):
                    node_states[node_name] = "replaced"
                    replaced.append("{} (pid {}, {})".format(
                        node_name, node_pid, node_uri))
                else:
                    node_states[node_name] = "running"

            if replaced:
                health = "node_replaced"
                reason = "same-name ROS node belongs to another process: " + "; ".join(
                    replaced)
            elif unverified:
                health = "node_unverified"
                reason = "could not verify node owner: " + "; ".join(unverified)
            elif any(state == "missing" for state in node_states.values()):
                elapsed = time.monotonic() - managed.get("started_monotonic", 0.0)
                missing = ", ".join(
                    node for node, state in node_states.items() if state == "missing")
                health = ("starting" if elapsed < self.node_startup_timeout_sec
                          else "missing_nodes")
                reason = "waiting for: " + missing if health == "starting" else (
                    "not registered: " + missing)
            else:
                health = "healthy"
                reason = ""
            self._set_node_health(key, managed, health, reason, node_states)

    def _set_node_health(self, key, managed, health, reason, node_states):
        managed["health"] = health
        managed["health_reason"] = reason
        managed["node_states"] = node_states

    def _stop_all(self, cleanup_ros=True):
        self._stop_processes(list(self.active))
        old_generation = self.generation_dir
        self.session_id = None
        self.reused_launches = {}
        if self.active:
            self._log("retaining generated configuration for processes not yet stopped")
        else:
            self.generation_dir = None
        self._write_registry()
        if not self.active and old_generation and old_generation.exists():
            shutil.rmtree(old_generation, ignore_errors=True)
        processes_stopped = not self.active
        if cleanup_ros:
            return processes_stopped and self._cleanup_ros_master()
        self.ros_cleanup_done = False
        return processes_stopped

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
            new_session = self.session_id != session_id
            profile = self.config["profiles"][profile_name]
            desired = self._desired_processes(
                profile_name, profile, vehicles, shared_launches)
            self._validate_expected_node_names(desired)

            if new_session:
                self._log("switching session from {} to {}; keeping matching live "
                          "launches and reconciling changes".format(
                              self.session_id or "(none)", session_id))
                self.ros_cleanup_done = False
            elif action == "start":
                self._log("start request for active session {}; reconciling idempotently".format(
                    session_id))
            self.session_id = session_id
            started, changed = self._reconcile(desired)
            if new_session or changed:
                self._refresh_node_status(force=True)
            elif action == "start":
                self._refresh_node_status()
            return {"ok": True, "action": action, "session_id": session_id,
                    "profile": profile_name, "active": self._status(),
                    "reused": self._reused_status()}
        if action == "heartbeat":
            return {"ok": True, "action": action, "session_id": self.session_id,
                    "active_count": len(self.active)}
        if action == "status":
            self._refresh_node_status(force=True)
            return {"ok": True, "session_id": self.session_id,
                    "active": self._status(),
                    "reused": self._reused_status()}
        if action == "stop":
            cleaned = self._stop_all()
            return {"ok": True, "action": "stop", "active": self._status(),
                    "cleanup_complete": cleaned}
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
                "health": managed.get("health", "unknown"),
                "health_reason": managed.get("health_reason", ""),
                "expected_nodes": managed.get("expected_nodes", []),
                "nodes": managed.get("node_states", {}),
                "log_path": record.get("log_path"),
            })
        return result

    def _reused_status(self):
        return [
            {"key": key, "label": reused.get("label"), "running": True,
             "health": "already_running",
             "health_reason": "reusing registered node group",
             "expected_nodes": reused.get("expected_nodes", [])}
            for key, reused in self.reused_launches.items()
        ]

    def _send(self, connection, value):
        raw = (json.dumps(value, ensure_ascii=False) + "\n").encode("utf-8")
        connection.sendall(raw)

    def _serve_connection(self, connection, address):
        self._log("connected host {}:{}".format(*address))
        buffer = bytearray()
        last_message = time.monotonic()
        connection.settimeout(1.0)
        should_close = False
        heartbeat_timed_out = False
        try:
            while not self.shutdown_event.is_set() and not should_close:
                if time.monotonic() - last_message > self.timeout_sec:
                    self._log("host heartbeat timeout; stopping active processes")
                    heartbeat_timed_out = True
                    break
                try:
                    chunk = connection.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    self._log("host disconnected; retaining processes for reconnect grace period")
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
                    self.last_client_activity = last_message
                    try:
                        message = json.loads(raw_line.decode("utf-8"))
                        if not isinstance(message, dict):
                            raise ValueError("each line must be a JSON object")
                        response = self._handle_message(message)
                        self._send(connection, response)
                        last_message = time.monotonic()
                        self.last_client_activity = last_message
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
            if heartbeat_timed_out:
                self._stop_all()

    def _expire_disconnected_session(self):
        if not (self.session_id is not None or self.active):
            return
        now = time.monotonic()
        if now - self.last_client_activity <= self.timeout_sec:
            return
        self._log("client disconnected past heartbeat timeout; stopping active processes")
        self.last_client_activity = now
        self._stop_all()
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
                    self._expire_disconnected_session()
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
    atexit.register(manager._stop_all)
    signal.signal(signal.SIGINT, manager.request_shutdown)
    signal.signal(signal.SIGTERM, manager.request_shutdown)
    signal.signal(signal.SIGHUP, manager.request_shutdown)
    try:
        manager.serve_forever()
    except Exception as error:
        print("[start-manager] fatal: {}".format(error), file=sys.stderr, flush=True)
        manager._stop_all()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
