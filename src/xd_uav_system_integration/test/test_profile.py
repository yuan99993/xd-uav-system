#!/usr/bin/env python3

import unittest

from xd_uav_system_integration.core import validate_profile
from xd_uav_system_integration.profile import (
    build_launch_plan,
    render_roslaunch,
    validate_component_profile,
)


class ProfileTest(unittest.TestCase):
    @staticmethod
    def profile():
        return {
            "schema_version": 1,
            "vehicles": [{"name": "uav1", "ego_id": 0}],
            "features": {
                "sead": {"enabled": True},
                "ego": {"enabled": True, "sensing": "external_pointcloud"},
            },
            "control": {"reference_owner": "sead"},
            "frames": {"common": "world"},
            "simulation": {"backend": "px4_gazebo", "use_sim_time": True},
        }

    def test_valid_profile(self):
        self.assertEqual(validate_profile(self.profile()), [])

    def test_rejects_unsafe_combinations(self):
        profile = self.profile()
        profile["control"]["reference_owner"] = "ego"
        profile["features"]["ego"]["enabled"] = False
        self.assertIn("ego owner requires ego.enabled", validate_profile(profile))

        profile = self.profile()
        profile["features"]["ego"]["sensing"] = "fake_drone"
        self.assertIn("px4_gazebo and fake_drone are mutually exclusive",
                      validate_profile(profile))

        profile = self.profile()
        profile["vehicles"] = [
            {"name": "uav1", "ego_id": 0},
            {"name": "uav2", "ego_id": 2},
        ]
        self.assertIn("ego_id values must be unique and continuous from 0",
                      validate_profile(profile))


class ComponentProfileTest(unittest.TestCase):
    @staticmethod
    def profile():
        return {
            "schema_version": 2,
            "vehicles": [{"name": "uav1", "ego_id": 0}],
            "frames": {"common": "world"},
            "simulation": {"backend": "px4_gazebo"},
            "control": {"initial_owner": "none"},
            "components": [
                {
                    "id": "control", "role": "controller",
                    "package": "integration", "launch": "control.launch",
                    "args": {"uav": "{vehicle.name}"},
                },
                {
                    "id": "ego", "role": "reference_source",
                    "source": "ego", "package": "integration",
                    "stage": "runtime",
                    "launch": "ego.launch", "requires": ["control"],
                    "args": {"id": "{vehicle.ego_id}"},
                },
                {
                    "id": "mux", "role": "reference_arbiter",
                    "package": "integration", "launch": "mux.launch",
                    "requires": ["control"],
                    "args": {"frame": "{frames.common}"},
                },
            ],
        }

    def test_builds_component_plan_and_roslaunch(self):
        profile = self.profile()
        self.assertEqual(validate_component_profile(profile), [])
        plan = build_launch_plan(profile)
        self.assertEqual(
            [item.component_id for item in plan],
            ["control:uav1", "ego:uav1", "mux:uav1"])
        self.assertEqual(plan[0].args["uav"], "uav1")
        self.assertEqual(plan[1].args["id"], "0")
        xml = render_roslaunch(plan)
        self.assertIn("$(find integration)/launch/control.launch", xml)
        self.assertIn('name="frame" value="world"', xml)
        self.assertNotIn("ego.launch", xml)
        runtime_xml = render_roslaunch(plan, "runtime")
        self.assertIn("ego.launch", runtime_xml)
        self.assertNotIn("control.launch", runtime_xml)

    def test_rejects_missing_dependency_and_owner_source(self):
        profile = self.profile()
        profile["components"][1]["enabled"] = False
        profile["control"]["initial_owner"] = "ego"
        errors = validate_component_profile(profile)
        self.assertIn(
            "initial_owner requires an enabled reference_source", errors)

    def test_rejects_duplicate_safety_roles(self):
        profile = self.profile()
        profile["components"].append({
            "id": "other_mux", "role": "reference_arbiter",
            "package": "integration", "launch": "mux.launch",
        })
        self.assertIn(
            "exactly one enabled reference_arbiter is required",
            validate_component_profile(profile))

if __name__ == "__main__":
    unittest.main()
