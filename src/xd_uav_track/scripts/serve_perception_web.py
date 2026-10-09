#!/usr/bin/env python3
"""Serve installed web assets; transport only, no detector or flight commands."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading

import rospkg
import rospy


class WebHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory, runtime, **kwargs):
        self.asset_root = Path(directory).resolve()
        self.runtime = runtime
        super().__init__(*args, directory=str(self.asset_root), **kwargs)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/perception_runtime.json":
            payload = json.dumps(self.runtime, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        super().do_GET()

    def send_head(self):
        candidate = Path(self.translate_path(self.path)).resolve()
        try:
            candidate.relative_to(self.asset_root)
        except ValueError:
            self.send_error(403, "asset outside web directory")
            return None
        # A deployed web directory is an allowlisted asset root, not a file browser.
        if candidate.is_dir():
            index = (candidate / "index.html").resolve()
            try:
                index.relative_to(self.asset_root)
            except ValueError:
                self.send_error(403, "index outside web directory")
                return None
            if not index.is_file():
                self.send_error(403, "directory listing disabled")
                return None
        return super().send_head()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

    def log_message(self, format, *args):
        rospy.logdebug("perception HTTP: " + format, *args)


def main():
    rospy.init_node("perception_web_http")
    address = str(rospy.get_param("~address", "127.0.0.1"))
    port = int(rospy.get_param("~port", 8080))
    if not 1 <= port <= 65535:
        raise ValueError("HTTP port must be within 1..65535")
    root = Path(rospkg.RosPack().get_path("xd_uav_track")) / "web"
    runtime = {
        "uav_name": str(rospy.get_param("~uav_name", "uav1")),
        "websocket_port": int(rospy.get_param("~websocket_port", 9090)),
    }
    if not 1 <= runtime["websocket_port"] <= 65535:
        raise ValueError("WebSocket port must be within 1..65535")
    server = ThreadingHTTPServer((address, port), partial(
        WebHandler, directory=str(root), runtime=runtime))
    server.daemon_threads = True
    worker = threading.Thread(target=server.serve_forever,
                              kwargs={"poll_interval": 0.2}, daemon=True)
    worker.start()
    rospy.loginfo("perception web: http://%s:%d (local/TLS proxy required for remote)",
                  address, port)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)


if __name__ == "__main__":
    main()
