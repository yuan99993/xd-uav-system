#!/usr/bin/env python3
"""Bounded real web/ROS/GPU switch regression; no synthetic detections.

This measures deployment transactions and video delivery, not ground-truth
tracking accuracy. Model rotations temporarily change the detector intentionally.
"""
import argparse
import asyncio
import base64
import json
import subprocess
import signal
import time
import uuid
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from tornado.websocket import websocket_connect
import rospy


class RosStartupDeadline(BaseException):
    """Not swallowed by rospy's generic Exception retry loops."""


def startup_deadline(_signal, _frame):
    raise RosStartupDeadline("ROS initialization exceeded 15 seconds")


class RuntimeCheck:
    def __init__(self, args):
        self.args = args
        self.root = "/" + args.uav.strip("/") + "/perception/"
        self.pending = {}
        self.counter = 0
        self.request_prefix = "runtime-" + uuid.uuid4().hex[:12] + "-"
        self.video = {}
        self.latency = {}
        self.state = None
        self.report = {"scope": "Gazebo/web transaction and video integration; not HOTA/IDF1",
                       "sources": {}, "switches": [], "gpu_memory_mib": [], "errors": []}

    async def read(self):
        while True:
            packet = await self.socket.read_message()
            if packet is None:
                raise RuntimeError("websocket disconnected")
            packet = json.loads(packet)
            if packet.get("op") == "service_response":
                future = self.pending.get(packet.get("id"))
                if future is not None and not future.done():
                    future.set_result(packet)
            elif packet.get("topic") == self.root + "command_result":
                result = packet["msg"]
                future = self.pending.get(result["request_id"])
                if result["stage"] in ("APPLIED", "FAILED", "REJECTED") and future is not None and not future.done():
                    future.set_result(result)
            elif packet.get("topic") == self.root + "state":
                self.state = packet["msg"]
            elif packet.get("topic") == self.root + "video":
                message = packet["msg"]
                targets = message["targets"]
                source = targets["image_source"]
                if source not in self.video and len(self.video) >= 8:
                    raise RuntimeError("source limit exceeded")
                self.video[source] = message
                stamp = targets["header"]["stamp"]
                age = rospy.Time.now().to_sec() - stamp["secs"] - stamp["nsecs"] * 1e-9
                if age >= 0:
                    self.latency.setdefault(source, deque(maxlen=4096)).append(age * 1000)
                summary = self.report["sources"].setdefault(source, {"frames": 0, "frames_with_targets": 0,
                    "native_dimensions": [], "measurement_sources": {}, "same_frame_errors": 0})
                summary["frames"] += 1
                summary["frames_with_targets"] += bool(targets["targets"])
                if message["image"]["header"]["stamp"] != targets["header"]["stamp"]:
                    summary["same_frame_errors"] += 1
                # Decode only the first frame/source. Do not make test monitoring
                # a second high-rate PNG decoder competing with the browser.
                if not summary["native_dimensions"]:
                    data = message["image"]["data"]
                    raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)
                    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    if image is None or image.shape[:2] != (targets["image_height"], targets["image_width"]):
                        raise RuntimeError("native pixel dimensions mismatch")
                    summary["native_dimensions"] = [image.shape[1], image.shape[0]]
                for target in targets["targets"]:
                    kind = target["measurement_source"]
                    summary["measurement_sources"][kind] = summary["measurement_sources"].get(kind, 0) + 1

    async def request(self, identifier, packet, seconds=25):
        future = asyncio.get_running_loop().create_future()
        self.pending[identifier] = future
        try:
            self.socket.write_message(json.dumps(packet))
            return await asyncio.wait_for(future, seconds)
        finally:
            self.pending.pop(identifier, None)

    async def state_get(self):
        self.counter += 1
        packet = await self.request("state-" + str(self.counter), dict(op="call_service",
            id="state-" + str(self.counter), service=self.root + "state/get", args={}))
        if not packet.get("result", False):
            raise RuntimeError("state service rejected")
        return packet["values"]["state"]

    async def command(self, suffix, **fields):
        state = await self.state_get()
        self.counter += 1
        identifier = self.request_prefix + str(self.counter)
        names = {"mode/set": "SetPerceptionModeCommand", "model/set": "SetDetectorModelCommand",
                 "target/select": "SelectPerceptionTargetCommand"}
        self.socket.write_message(json.dumps(dict(op="advertise", topic=self.root + suffix,
            type="xd_uav_track/" + names[suffix])))
        message = dict(request_id=identifier, session_id=state["session_id"],
            expected_state_version=state["state_version"], issued_at=state["header"]["stamp"], ttl_sec=20, **fields)
        started = time.monotonic()
        result = await self.request(identifier, dict(op="publish", topic=self.root + suffix, msg=message))
        if result["stage"] != "APPLIED":
            raise RuntimeError(str(result))
        return time.monotonic() - started, await self.state_get()

    def memory(self):
        result = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                                capture_output=True, text=True, timeout=3, check=True)
        self.report["gpu_memory_mib"].append(int(result.stdout.splitlines()[0]))

    async def run(self):
        connection_deadline = time.monotonic() + 20
        while True:
            try:
                self.socket = await websocket_connect(self.args.url, connect_timeout=2,
                                                     max_message_size=33554432)
                break
            except Exception:
                if time.monotonic() >= connection_deadline:
                    raise
                await asyncio.sleep(.2)
        reader = asyncio.create_task(self.read())
        try:
            for suffix in ("video", "state", "command_result"):
                self.socket.write_message(json.dumps(dict(op="subscribe", topic=self.root + suffix,
                    queue_length=32 if suffix == "command_result" else 1)))
            deadline = time.monotonic() + 40
            while not self.state or not self.state["model_ready"] or not self.video:
                if reader.done():
                    reader.result()
                if time.monotonic() > deadline:
                    raise RuntimeError("ready/video timeout")
                await asyncio.sleep(.1)
            for mode in ("track", "fusion", "detect"):
                _, state = await self.command("mode/set", mode=mode)
                if state["mode"] != mode:
                    raise RuntimeError("mode did not become effective")
            self.memory()
            state = await self.state_get()
            for index in range(self.args.rotations):
                models = ("train7", "train4", "train")
                model = models[(models.index(state["active_model_id"]) + 1) % len(models)]
                previous_epoch = state["model_epoch"]
                elapsed, state = await self.command("model/set", model_id=model)
                if state["active_model_id"] != model or state["model_epoch"] != previous_epoch + 1:
                    raise RuntimeError("model/epoch commit mismatch")
                self.report["switches"].append({"model_id": model, "epoch": state["model_epoch"], "seconds": elapsed})
                self.memory()
                if (index + 1) % 10 == 0:
                    print("completed model rotations:", index + 1, flush=True)
            if state["active_model_id"] != "train7":
                await self.command("model/set", model_id="train7")
            if self.args.select_target:
                await self.command("mode/set", mode="fusion")
                deadline = time.monotonic() + 10
                candidate = None
                while candidate is None:
                    state = await self.state_get()
                    for frame in self.video.values():
                        snapshot = frame["targets"]
                        if snapshot["state_version"] != state["state_version"]:
                            continue
                        candidate = next((t for t in snapshot["targets"] if t["lifecycle_state"] == "confirmed"
                                          and t["measurement_source"] == "detector"), None)
                        if candidate is not None:
                            break
                    if time.monotonic() > deadline:
                        raise RuntimeError("no fresh confirmed target for selection")
                    if candidate is None:
                        await asyncio.sleep(.1)
                _, state = await self.command("target/select", selection_type="track_id",
                    target_uid=candidate["target_uid"], track_id=candidate["track_id"],
                    image_source=snapshot["image_source"], frame_seq=snapshot["frame_seq"],
                    capture_stamp=snapshot["header"]["stamp"], point_norm=[0., 0.],
                    roi_norm=[0., 0., 0., 0.], tracker_algorithm="auto")
                if not state["selection_active"] or state["selected_target_uid"] != candidate["target_uid"]:
                    raise RuntimeError("selected entity mismatch")
                self.report["selected_target_uid"] = state["selected_target_uid"]
                self.report["selected_algorithm"] = state["active_tracker_algorithm"]
            await asyncio.sleep(self.args.observe_seconds)
            if reader.done():
                reader.result()
            memory = self.report["gpu_memory_mib"]
            growth = (float(np.mean(memory[-9:])) - float(np.mean(memory[10:19]))
                      if len(memory) >= 31 else None)
            self.report["post_warmup_memory_growth_mib"] = growth
            self.report["memory_growth_limit_mib"] = 64
            self.report["passed"] = (len(self.report["sources"]) >= self.args.required_sources and
                all(s["frames"] > 0 and s["frames_with_targets"] > 0 and s["same_frame_errors"] == 0
                    for s in self.report["sources"].values()) and (growth is None or growth <= 64))
            self.report["identity_accuracy_certified"] = False
            self.report["video_capture_to_ws_ms"] = {source: {
                "samples": len(samples), "p50": float(np.percentile(samples, 50)),
                "p95": float(np.percentile(samples, 95)), "max": max(samples)}
                for source, samples in self.latency.items() if samples}
            if self.args.select_target:
                final_state = await self.state_get()
                if final_state["selected_target_uid"] != self.report["selected_target_uid"]:
                    raise RuntimeError("public selected identity changed")
                await self.command("target/select", selection_type="clear", image_source="",
                    target_uid="", track_id=-1, frame_seq=0, capture_stamp={"secs": 0, "nsecs": 0},
                    point_norm=[0., 0.], roi_norm=[0., 0., 0., 0.], tracker_algorithm="auto")
                await self.command("mode/set", mode="detect")
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            self.socket.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="ws://127.0.0.1:9090")
    parser.add_argument("--uav", default="moving_uav_search")
    parser.add_argument("--rotations", type=int, default=100)
    parser.add_argument("--observe-seconds", type=float, default=15)
    parser.add_argument("--required-sources", type=int, default=2)
    parser.add_argument("--select-target", action="store_true")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 0 <= args.rotations <= 100 or not 0 <= args.observe_seconds <= 60:
        parser.error("rotations must be 0..100; observe seconds 0..60")
    check = RuntimeCheck(args)
    try:
        previous_handler = signal.signal(signal.SIGALRM, startup_deadline)
        signal.alarm(15)
        try:
            rospy.init_node("perception_runtime_monitor", argv=["perception_runtime_monitor"], disable_signals=True)
        finally:
            signal.alarm(0)
            signal.signal(signal.SIGALRM, previous_handler)
        asyncio.run(asyncio.wait_for(check.run(), timeout=240))
    except (Exception, RosStartupDeadline) as error:
        check.report["passed"] = False
        check.report["errors"].append(str(error))
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(check.report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in check.report.items() if key != "switches"}, indent=2))
    raise SystemExit(0 if check.report.get("passed") else 1)


if __name__ == "__main__":
    main()
