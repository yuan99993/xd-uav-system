#!/usr/bin/env python3

import os
import unittest

import yaml


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class PortableConfigurationTest(unittest.TestCase):

    def test_single_gazebo_world_is_fixed_to_local_origin(self):
        path = os.path.join(PACKAGE_DIR, "config", "world_registration.yaml")
        with open(path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["world"]["datum"]["latitude"], 47.397743)
        self.assertEqual(config["world"]["datum"]["longitude"], 8.545594)
        vehicle = config["vehicle_configs"]["uav1"]
        self.assertEqual(vehicle["mode"], "fixed")
        self.assertEqual(vehicle["fixed_position"], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
