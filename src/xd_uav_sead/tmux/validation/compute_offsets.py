#!/usr/bin/env python3
import os
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def wait_for_message(topic, message_type, total_timeout=180.0):
    deadline = rospy.get_time() + total_timeout
    attempts = 0
    while not rospy.is_shutdown():
        attempts += 1
        try:
            return rospy.wait_for_message(topic, message_type, timeout=5.0)
        except rospy.ROSException:
            # rospy.get_time() is simulated time here and may pause during startup,
            # so also bound retries independently.
            if attempts >= int(total_timeout / 5.0) or rospy.get_time() >= deadline:
                published = any(name == topic for name, _ in rospy.get_published_topics())
                raise RuntimeError(
                    "timed out waiting for {} after {:.0f}s (published={})".format(
                        topic, total_timeout, published
                    )
                )
            print("waiting for {} ({:.0f}s elapsed)...".format(topic, attempts * 5.0))


def main():
    output = os.environ.get(
        "SEAD_VALIDATION_RUNTIME",
        "/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/sead_validation_offsets.env",
    )
    run_id = os.environ.get("SEAD_VALIDATION_RUN_ID")
    if not run_id:
        raise RuntimeError("SEAD_VALIDATION_RUN_ID is missing")
    rospy.init_node("sead_validation_compute_offsets", anonymous=True)
    print("waiting for Gazebo models uav1/uav2/uav3...")
    models = None
    for attempt in range(36):
        candidate = wait_for_message("/gazebo/model_states", ModelStates, total_timeout=10.0)
        missing = [name for name in ("uav1", "uav2", "uav3") if name not in candidate.name]
        if not missing:
            models = candidate
            break
        print("waiting for Gazebo models: {} ({}s elapsed)...".format(", ".join(missing), (attempt + 1) * 5))
        rospy.sleep(5.0)
    if models is None:
        raise RuntimeError("timed out waiting for all three Gazebo models")

    odometry = {}
    for index in (1, 2, 3):
        name = "uav{}".format(index)
        topic = "/{}/mavros/local_position/odom".format(name)
        print("waiting for {}...".format(topic))
        odometry[name] = wait_for_message(topic, Odometry).pose.pose.position

    # Refresh the world poses after all three MAVROS streams are live so the
    # offsets belong to the same completed spawn, not an early partial snapshot.
    models = wait_for_message("/gazebo/model_states", ModelStates)
    lines = []
    for index in (1, 2, 3):
        name = "uav{}".format(index)
        if name not in models.name:
            raise RuntimeError("Gazebo model {} not found; found {}".format(name, models.name))
        world = models.pose[models.name.index(name)].position
        odom = odometry[name]
        values = (world.x - odom.x, world.y - odom.y, world.z - odom.z)
        lines.extend(
            [
                "U{}_X={:.9f}".format(index, values[0]),
                "U{}_Y={:.9f}".format(index, values[1]),
                "U{}_Z={:.9f}".format(index, values[2]),
            ]
        )
        print("{} offset = ({:.6f}, {:.6f}, {:.6f})".format(name, *values))
    lines.insert(0, "VALIDATION_RUN_ID={}".format(run_id))
    os.makedirs(os.path.dirname(output), exist_ok=True)
    temporary = output + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")
    os.replace(temporary, output)
    print("offset file written: {}".format(output))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("auto_offsets failed: {}".format(exc), file=sys.stderr)
        sys.exit(1)
