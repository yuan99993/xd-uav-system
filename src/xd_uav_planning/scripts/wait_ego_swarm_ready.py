#!/usr/bin/env python3
"""Wait until every EGO swarm vehicle proves a live closed control loop."""

import argparse
import math
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from xd_uav_controller.msg import ControlCommand


class VehicleEvidence:
    def __init__(self, name):
        self.name = name
        self.connected = False
        self.armed = False
        self.healthy = False
        self.owner = "none"
        self.forwarding = False
        self.command_valid = False
        self.initial_xy = None
        self.maximum_motion = 0.0
        self.last_update = {}

    def mark(self, key):
        self.last_update[key] = time.monotonic()

    def state_cb(self, message):
        self.connected = bool(message.connected)
        self.armed = bool(message.armed)
        self.mark("state")

    def health_cb(self, message):
        self.healthy = bool(message.data)
        self.mark("health")

    def owner_cb(self, message):
        self.owner = message.data
        self.mark("owner")

    def diagnostics_cb(self, message):
        matching = [item for item in message.status
                    if item.name.endswith("/reference_mux")]
        if matching:
            status = matching[-1]
            values = {item.key: item.value for item in status.values}
            self.forwarding = (status.level == DiagnosticStatus.OK and
                               status.message == "forwarding" and
                               values.get("owner") == "ego" and
                               values.get("reason") == "ok")
            self.mark("diagnostics")

    def command_cb(self, message):
        self.command_valid = bool(message.valid)
        self.mark("command")

    def odom_cb(self, message):
        point = message.pose.pose.position
        xy = (float(point.x), float(point.y))
        if self.initial_xy is None:
            self.initial_xy = xy
        self.maximum_motion = max(
            self.maximum_motion,
            math.hypot(xy[0] - self.initial_xy[0],
                       xy[1] - self.initial_xy[1]))
        self.mark("odom")

    def missing(self, now, freshness, minimum_motion):
        missing = []
        for key in ("state", "health", "owner", "diagnostics", "command", "odom"):
            if now - self.last_update.get(key, -1e9) > freshness:
                missing.append(key + "_stale")
        if not self.connected:
            missing.append("mavros_disconnected")
        if not self.armed:
            missing.append("not_armed")
        if not self.healthy:
            missing.append("ego_unhealthy")
        if self.owner != "ego":
            missing.append("owner=" + self.owner)
        if not self.forwarding:
            missing.append("mux_not_forwarding")
        if not self.command_valid:
            missing.append("controller_command_invalid")
        if self.maximum_motion < minimum_motion:
            missing.append("motion={:.2f}m".format(self.maximum_motion))
        return missing


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--minimum-horizontal-motion", type=float, default=0.50)
    parser.add_argument("--freshness", type=float, default=1.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("wait_ego_swarm_ready", anonymous=True)
    vehicles = {name: VehicleEvidence(name)
                for name in ("uav1", "uav2", "uav3")}
    subscribers = []
    for name, evidence in vehicles.items():
        subscribers.extend([
            rospy.Subscriber("/{}/mavros/state".format(name), State,
                             evidence.state_cb, queue_size=1),
            rospy.Subscriber("/{}/ego/system_healthy".format(name), Bool,
                             evidence.health_cb, queue_size=1),
            rospy.Subscriber("/{}/integration/reference_owner".format(name),
                             String, evidence.owner_cb, queue_size=1),
            rospy.Subscriber("/{}/integration/mux_diagnostics".format(name),
                             DiagnosticArray, evidence.diagnostics_cb,
                             queue_size=1),
            rospy.Subscriber("/{}/controller/command".format(name),
                             ControlCommand, evidence.command_cb, queue_size=2),
            rospy.Subscriber("/{}/mavros/local_position/odom".format(name),
                             Odometry, evidence.odom_cb, queue_size=2),
        ])
    deadline = time.monotonic() + args.timeout
    next_report = 0.0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        now = time.monotonic()
        failures = {
            name: evidence.missing(
                now, args.freshness, args.minimum_horizontal_motion)
            for name, evidence in vehicles.items()}
        if all(not missing for missing in failures.values()):
            rospy.loginfo("closed-loop swarm readiness passed: %s",
                          ", ".join("{} motion={:.2f}m".format(
                              name, vehicles[name].maximum_motion)
                                    for name in vehicles))
            return 0
        if now >= next_report:
            print("readiness: " + "; ".join(
                "{} [{}]".format(name, ",".join(missing) or "ok")
                for name, missing in failures.items()), flush=True)
            next_report = now + 5.0
        rate.sleep()
    print("swarm readiness timeout: " + "; ".join(
        "{} [{}]".format(name, ",".join(evidence.missing(
            time.monotonic(), args.freshness,
            args.minimum_horizontal_motion)))
        for name, evidence in vehicles.items()), file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
