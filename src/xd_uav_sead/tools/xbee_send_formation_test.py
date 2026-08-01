#!/usr/bin/env python3
"""
Send formation test commands from GCS XBee to UAV XBee nodes.

Sequence:
1) Swarm_Command (msg_id=24): update formation config
2) Formation_Point (msg_id=26): command rally/formation target point

Default UART: /dev/ttyUSB0 @ 57600
"""

import argparse
import struct
import time
from typing import Dict, Tuple

from digi.xbee.devices import DigiMeshDevice, RemoteDigiMeshDevice, XBee64BitAddress


MSG_SWARM_COMMAND = 24
MSG_FORMATION_POINT = 26

SHAPE_CODES = {
    "VEE": 1,
    "ECHELON_LEFT": 2,
    "ECHELON_RIGHT": 3,
    "TRAIL": 4,
    "TRIANGLE": 5,
    "WEDGE_WIDE": 6,
    "ARROW": 7,
    "INVERTED_VEE": 8,

}

UAV_MACS: Dict[int, str] = {
    1: "0013A2004126C97C",
    2: "0013A2004154C5D3",
    3: "0013A2004105EB5A",
}


def pack_swarm_command(
    uav_to: int,
    enable: int,
    shape: int,
    leader_id: int,
    spacing: float,
    standoff: float,
    safe_sep: float,
    alt_step: float,
    desired_target_time: float,
) -> bytes:
    return struct.pack(
        "<BBBBBiiiid",
        MSG_SWARM_COMMAND,
        int(uav_to) & 0xFF,
        int(enable) & 0xFF,
        int(shape) & 0xFF,
        int(leader_id) & 0xFF,
        int(spacing * 1e3),
        int(standoff * 1e3),
        int(safe_sep * 1e3),
        int(alt_step * 1e3),
        float(desired_target_time),
    )


def pack_formation_point(
    uav_to: int,
    point_id: int,
    e: float,
    n: float,
    u: float,
    loiter_radius: float,
) -> bytes:
    return struct.pack(
        "<BBHiiii",
        MSG_FORMATION_POINT,
        int(uav_to) & 0xFF,
        int(point_id) & 0xFFFF,
        int(e * 1e3),
        int(n * 1e3),
        int(u * 1e3),
        int(loiter_radius * 1e3),
    )


def parse_uav_ids(text: str) -> Tuple[int, ...]:
    ids = []
    for x in text.split(","):
        x = x.strip()
        if not x:
            continue
        ids.append(int(x))
    return tuple(ids)


def send_unicast(device: DigiMeshDevice, mac: str, payload: bytes, label: str) -> None:
    remote = RemoteDigiMeshDevice(device, XBee64BitAddress.from_hex_string(mac))
    device.send_data(remote, payload)
    print(f"[TX] {label} -> {mac} len={len(payload)} hex={payload.hex(' ')}")


def main() -> None:
    parser = argparse.ArgumentParser(description="XBee formation command sender")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="XBee serial port")
    parser.add_argument("--baud", type=int, default=57600, help="XBee baudrate")
    parser.add_argument("--uavs", default="1,2,3", help="Target UAV IDs, e.g. 1,2,3")
    parser.add_argument("--mode", choices=["unicast", "broadcast"], default="unicast")

    parser.add_argument("--enable", type=int, default=1, help="formation enable (0/1)")
    parser.add_argument("--shape", default="TRAIL", choices=list(SHAPE_CODES.keys()))
    parser.add_argument("--leader-id", type=int, default=1)
    parser.add_argument("--spacing", type=float, default=70.0)
    parser.add_argument("--standoff", type=float, default=5000.0)
    parser.add_argument("--safe-sep", type=float, default=45.0)
    parser.add_argument("--alt-step", type=float, default=20.0)
    parser.add_argument("--desired-target-time", type=float, default=0.0)

    parser.add_argument("--point-id", type=int, default=1)
    parser.add_argument("--target-e", type=float, default=800.0, help="ENU East meters")
    parser.add_argument("--target-n", type=float, default=0.0, help="ENU North meters")
    parser.add_argument("--target-u", type=float, default=120.0, help="ENU Up meters")
    parser.add_argument("--loiter-radius", type=float, default=120.0)

    parser.add_argument("--repeat", type=int, default=1, help="Repeat each command N times")
    parser.add_argument("--gap", type=float, default=0.25, help="Seconds between sends")

    args = parser.parse_args()

    uav_ids = parse_uav_ids(args.uavs)
    for uid in uav_ids:
        if uid not in UAV_MACS:
            raise ValueError(f"UAV ID {uid} has no MAC in UAV_MACS")

    shape_code = SHAPE_CODES[args.shape]
    device = DigiMeshDevice(args.port, args.baud)

    print(f"[INFO] Open XBee: {args.port} @ {args.baud}")
    device.open()
    try:
        print(f"[INFO] Local Node ID: {device.get_node_id()}")
        print(f"[INFO] Local 64-bit : {device.get_64bit_addr()}")
        print(f"[INFO] Targets      : {uav_ids} mode={args.mode}")

        if args.mode == "broadcast":
            swarm_pkt = pack_swarm_command(
                uav_to=0,
                enable=args.enable,
                shape=shape_code,
                leader_id=args.leader_id,
                spacing=args.spacing,
                standoff=args.standoff,
                safe_sep=args.safe_sep,
                alt_step=args.alt_step,
                desired_target_time=args.desired_target_time,
            )
            form_pkt = pack_formation_point(
                uav_to=0,
                point_id=args.point_id,
                e=args.target_e,
                n=args.target_n,
                u=args.target_u,
                loiter_radius=args.loiter_radius,
            )

            for i in range(max(1, args.repeat)):
                device.send_data_broadcast(swarm_pkt)
                print(f"[TX] Swarm_Command broadcast #{i+1} len={len(swarm_pkt)}")
                time.sleep(args.gap)
                device.send_data_broadcast(form_pkt)
                print(f"[TX] Formation_Point broadcast #{i+1} len={len(form_pkt)}")
                time.sleep(args.gap)
        else:
            for uid in uav_ids:
                mac = UAV_MACS[uid]
                swarm_pkt = pack_swarm_command(
                    uav_to=uid,
                    enable=args.enable,
                    shape=shape_code,
                    leader_id=args.leader_id,
                    spacing=args.spacing,
                    standoff=args.standoff,
                    safe_sep=args.safe_sep,
                    alt_step=args.alt_step,
                    desired_target_time=args.desired_target_time,
                )
                form_pkt = pack_formation_point(
                    uav_to=uid,
                    point_id=args.point_id,
                    e=args.target_e,
                    n=args.target_n,
                    u=args.target_u,
                    loiter_radius=args.loiter_radius,
                )

                for i in range(max(1, args.repeat)):
                    send_unicast(device, mac, swarm_pkt, f"Swarm_Command uav={uid} #{i+1}")
                    time.sleep(args.gap)
                    send_unicast(device, mac, form_pkt, f"Formation_Point uav={uid} #{i+1}")
                    time.sleep(args.gap)

        print("[DONE] Commands sent.")

    finally:
        if device.is_open():
            device.close()
            print("[INFO] XBee closed.")


if __name__ == "__main__":
    main()

