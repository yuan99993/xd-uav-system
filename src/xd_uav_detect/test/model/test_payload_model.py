#!/usr/bin/env python3

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[2]


def test_shared_payload_snapshots_and_contract():
    subprocess.run(
        [sys.executable,
         str(PACKAGE_DIR / "scripts/model/render_payload_models.py")],
        check=True,
    )

    common_import = "common/eo_gimbal.sdf.jinja"
    for template in (
            PACKAGE_DIR / "models/x500_gimbal/x500_gimbal.sdf.jinja",
            PACKAGE_DIR / "models/plane_gimbal/plane_gimbal.sdf.jinja"):
        assert common_import in template.read_text(encoding="utf-8")

    for snapshot in (
            PACKAGE_DIR / "models/x500_gimbal/model.sdf",
            PACKAGE_DIR / "models/x500_gimbal/sensor_demo.sdf",
            PACKAGE_DIR / "models/plane_gimbal/plane.sdf"):
        root = ET.parse(str(snapshot)).getroot()
        model = root.find("model")
        assert model is not None
        pitch = model.find("link[@name='gimbal_pitch']")
        assert pitch is not None
        assert pitch.find("sensor[@name='gimbal_camera']") is not None
        assert pitch.find("sensor[@name='laser_range']") is not None
        assert model.find("joint[@name='gimbal_yaw_joint']") is not None
        assert model.find("joint[@name='gimbal_pitch_joint']") is not None
        plugin = model.find("plugin[@name='gimbal_joint_trajectory']")
        assert plugin is not None
        assert plugin.get("filename") == (
            "libxd_uav_detect_gimbal_joint_controller.so")
