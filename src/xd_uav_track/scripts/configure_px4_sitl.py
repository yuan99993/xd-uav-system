#!/usr/bin/env python3
"""Apply the PX4 parameters required by the fixed-wing SITL harness.

The script is deliberately optional and only changes parameters in the
namespaced MAVROS instance.  It waits for MAVROS, so the launch file can start
it alongside PX4/Gazebo without introducing a startup race.
"""

import argparse
import time

import rospy
from mavros_msgs.srv import (
    ParamGet,
    ParamGetRequest,
    ParamPull,
    ParamPullRequest,
    ParamSet,
    ParamSetRequest,
)


PARAMETERS = {
    # NAV_RCL_ACT has no disabled value in PX4 v1.13.  Use Hold as the
    # well-defined fallback and exempt Hold/Offboard in this RC-less harness.
    "NAV_RCL_ACT": 1,
    "NAV_DLL_ACT": 0,
    "COM_RC_IN_MODE": 4,
    "COM_RCL_EXCEPT": 6,
    "COM_OBL_ACT": 0,
    "COM_OBL_RC_ACT": 0,
    "ASPD_PRIMARY": 1,
    "ASPD_DO_CHECKS": 7,
    "FW_ARSP_MODE": 0,
    "CBRK_AIRSPD_CHK": 0,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/uav1")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("configure_px4_sitl", anonymous=True)
    namespace = "/" + args.namespace.strip("/")
    param_prefix = namespace + "/mavros/param"
    pull_name = param_prefix + "/pull"
    get_name = param_prefix + "/get"
    set_name = param_prefix + "/set"
    try:
        # MAVROS advertises the parameter services before its cache has been
        # populated from PX4.  Pull explicitly, then use ParamGet as the
        # readiness gate; calling ParamSet immediately after service discovery
        # otherwise produces a transient "Unknown parameter" for every key.
        rospy.wait_for_service(pull_name, timeout=60.0)
        rospy.wait_for_service(get_name, timeout=60.0)
        rospy.wait_for_service(set_name, timeout=60.0)
    except rospy.ROSException as exc:
        rospy.logerr("PX4 parameter service unavailable: %s", exc)
        return 1

    puller = rospy.ServiceProxy(pull_name, ParamPull)
    getter = rospy.ServiceProxy(get_name, ParamGet)
    setter = rospy.ServiceProxy(set_name, ParamSet)

    try:
        pull_response = puller(ParamPullRequest(force_pull=True))
        if not pull_response.success:
            rospy.logwarn("PX4 parameter pull was not accepted; waiting for cache anyway")
        else:
            rospy.loginfo("PX4 parameter pull requested (%d parameters)",
                          pull_response.param_received)
    except rospy.ServiceException as exc:
        rospy.logwarn("PX4 parameter pull failed; waiting for cache: %s", exc)

    # Wait until each parameter is known to MAVROS.  A few parameters differ
    # between PX4 releases, so unsupported keys are reported and skipped while
    # the available keys are still applied.
    available = []
    unsupported = []
    deadline = time.monotonic() + 45.0
    pending = list(PARAMETERS)
    while pending and not rospy.is_shutdown() and time.monotonic() < deadline:
        remaining = []
        for name in pending:
            request = ParamGetRequest(param_id=name)
            try:
                response = getter(request)
            except rospy.ServiceException:
                remaining.append(name)
                continue
            if response.success:
                available.append(name)
            else:
                remaining.append(name)
        pending = remaining
        if pending:
            rospy.sleep(0.5)

    if pending:
        unsupported.extend(pending)
        rospy.logwarn("PX4 parameters unavailable after sync: %s",
                      ", ".join(pending))

    failed = []
    for name in available:
        value = PARAMETERS[name]
        request = ParamSetRequest()
        request.param_id = name
        request.value.integer = int(value)
        request.value.real = float(value)
        try:
            response = setter(request)
        except rospy.ServiceException as exc:
            rospy.logerr("failed to set %s: %s", name, exc)
            failed.append(name)
            continue
        if not response.success:
            rospy.logerr("PX4 rejected %s=%s", name, value)
            failed.append(name)
        else:
            rospy.loginfo("PX4 SITL parameter %s=%s", name, value)

    if failed:
        rospy.logerr("PX4 SITL parameter setup failed: %s", ", ".join(failed))
        return 1
    if not available:
        rospy.logerr("PX4 SITL parameter setup failed: no requested parameters available")
        return 1
    if unsupported:
        rospy.logwarn("PX4 SITL configured with %d/%d requested parameters",
                      len(available), len(PARAMETERS))
    else:
        rospy.loginfo("PX4 SITL airspeed/failsafe parameters configured")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
