#!/usr/bin/env python3
"""Example TCP client: send a JSON state file and keep it alive with heartbeats."""

import argparse
import json
import os
import socket
import sys
import time


def send_message(stream, value):
    stream.write((json.dumps(value, ensure_ascii=False) + "\n").encode("utf-8"))
    stream.flush()
    line = stream.readline()
    if not line:
        raise ConnectionError("start manager closed the connection")
    result = json.loads(line.decode("utf-8"))
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if not result.get("ok"):
        raise RuntimeError(result.get("error", "request failed"))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state_json", help="file containing a start/update snapshot")
    parser.add_argument("--host", required=True, help="IP of the aircraft-code host")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--heartbeat", type=float, default=5.0)
    args = parser.parse_args()

    with open(args.state_json, "r", encoding="utf-8") as state_file:
        state = json.load(state_file)
    if state.get("action") not in ("start", "update", "state"):
        state["action"] = "start"
    token = os.environ.get("XD_UAV_START_TOKEN")
    if token:
        state["token"] = token

    sock = socket.create_connection((args.host, args.port), timeout=10.0)
    stream = sock.makefile("rwb")
    try:
        send_message(stream, state)
        session_id = state["session_id"]
        while True:
            time.sleep(args.heartbeat)
            heartbeat = {"action": "heartbeat", "session_id": session_id}
            if token:
                heartbeat["token"] = token
            send_message(stream, heartbeat)
    except KeyboardInterrupt:
        try:
            stop = {"action": "stop"}
            if token:
                stop["token"] = token
            send_message(stream, stop)
        except Exception as error:
            print("could not send stop request: {}".format(error), file=sys.stderr)
    finally:
        stream.close()
        sock.close()


if __name__ == "__main__":
    main()
