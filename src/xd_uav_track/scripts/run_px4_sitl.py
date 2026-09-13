#!/usr/bin/env python3
"""Exec PX4 SITL from an explicit checkout for the ROS launch wrapper."""

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--px4-autopilot-dir", required=True)
    # roslaunch appends its own __name/__log remaps after the node arguments;
    # keep PX4 switches such as -d while discarding only those ROS remaps.
    args, passthrough = parser.parse_known_args()
    root = os.path.abspath(args.px4_autopilot_dir)
    binary = os.path.join(root, "build", "px4_sitl_default", "bin", "px4")
    romfs = os.path.join(root, "build", "px4_sitl_default", "etc")
    rc_script = os.path.join(romfs, "init.d-posix", "rcS")
    if not os.path.isfile(binary) or not os.access(binary, os.X_OK):
        parser.error("PX4 SITL binary is not executable: %s" % binary)
    if not os.path.isdir(romfs) or not os.path.isfile(rc_script):
        parser.error("PX4 SITL ROMFS/init script is incomplete under: %s" % root)
    passthrough = [argument for argument in passthrough
                   if not argument.startswith("__name:=") and
                   not argument.startswith("__log:=")]
    os.execv(binary, [binary, romfs, "-s", rc_script] + passthrough)


if __name__ == "__main__":
    main()
