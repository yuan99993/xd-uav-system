#!/usr/bin/env python3
"""Run upstream rosbridge with an explicitly bound listening interface.

Some Noetic rosbridge versions set the advertised URL's address but omit the
interface argument to Autobahn listenWS, unintentionally listening on all NICs.
Keep upstream protocol/security configuration; only fix the TCP bind call.
"""
import runpy
from pathlib import Path

import rospkg
import rospy
import autobahn.twisted.websocket as websocket


def main():
    original = websocket.listenWS

    def bound_listen(factory, contextFactory=None, backlog=50, interface=""):
        address = str(rospy.get_param("~address", "127.0.0.1")).strip()
        if not address:
            raise ValueError("rosbridge address must be explicit; use 0.0.0.0 deliberately")
        return original(factory, contextFactory, backlog, interface=address)

    websocket.listenWS = bound_listen
    package = Path(rospkg.RosPack().get_path("rosbridge_server"))
    # ROS packages may live in share/ while executable scripts live in lib/.
    from roslib.packages import find_node
    nodes = find_node("rosbridge_server", "rosbridge_websocket")
    if not nodes:
        raise RuntimeError("rosbridge_server executable not installed: " + str(package))
    try:
        runpy.run_path(nodes[0], run_name="__main__")
    finally:
        websocket.listenWS = original


if __name__ == "__main__":
    main()
