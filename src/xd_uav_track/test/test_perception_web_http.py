#!/usr/bin/env python3
"""Bounded HTTP asset/runtime check; no ROS master, detector, or GPU."""
from functools import partial
from http.server import ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import urlopen
from unittest.mock import patch


script = Path(__file__).resolve().parents[1] / "scripts/serve_perception_web.py"
spec = importlib.util.spec_from_file_location("perception_http_test", script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WebHttpTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "web"
        self.directory.mkdir()
        (self.directory / "index.html").write_text("<html>test</html>", encoding="utf-8")
        self.runtime = {"uav_name": "test_uav", "websocket_port": 12345}
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), partial(
            module.WebHandler, directory=str(self.directory), runtime=self.runtime))
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.addCleanup(self.stop)
        self.url = "http://127.0.0.1:" + str(self.server.server_port)

    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
        self.assertFalse(self.thread.is_alive())

    def test_runtime_and_assets_are_uncached_and_dimension_neutral(self):
        with patch.object(module.rospy, "logdebug"):
            with urlopen(self.url + "/perception_runtime.json", timeout=2) as response:
                self.assertEqual(self.runtime, json.load(response))
                self.assertEqual("no-store", response.headers["Cache-Control"])
            with urlopen(self.url + "/", timeout=2) as response:
                self.assertEqual(b"<html>test</html>", response.read())

    def test_directory_listing_and_escaping_symlinks_are_rejected(self):
        (self.directory / "no-index").mkdir()
        secret = Path(self.temporary.name) / "outside.txt"
        secret.write_text("private", encoding="utf-8")
        (self.directory / "outside.txt").symlink_to(secret)
        (self.directory / "index-escape").mkdir()
        (self.directory / "index-escape/index.html").symlink_to(secret)
        with patch.object(module.rospy, "logdebug"):
            for route in ("/no-index/", "/outside.txt", "/index-escape/"):
                with self.assertRaises(HTTPError) as error:
                    urlopen(self.url + route, timeout=2)
                self.assertEqual(403, error.exception.code)


if __name__ == "__main__":
    unittest.main()
