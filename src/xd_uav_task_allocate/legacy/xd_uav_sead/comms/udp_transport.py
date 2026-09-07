#!/usr/bin/env python3

"""Raw UDP transport for the existing SEAD/XBee binary protocol.

The ground station already sends the same PacketProtocol payload over XBee and
UDP.  This adapter intentionally does not add an envelope or reinterpret the
payload: every UDP datagram is one complete SEAD binary packet.
"""

from dataclasses import dataclass
import socket
from typing import Iterable, Optional, Tuple

import rospy

from xd_uav_sead.comms.rosbridge import FakePacket


@dataclass(frozen=True)
class UdpEndpoint:
    name: str
    host: str
    port: int
    uav_id: Optional[int] = None

    @property
    def socket_address(self) -> Tuple[str, int]:
        return self.host, self.port


class RawUdpTransport:
    """XBee-compatible API backed by unmodified UDP datagrams."""

    def __init__(self, uav_id: int):
        self.uav_id = int(uav_id)
        bind_host = str(rospy.get_param("~communication/udp/bind_host", "0.0.0.0"))
        bind_port = int(rospy.get_param("~communication/udp/bind_port", 15660))
        self.max_datagram_size = int(
            rospy.get_param("~communication/udp/max_datagram_size", 65507)
        )
        self.allowed_senders = {
            str(value)
            for value in rospy.get_param("~communication/udp/allowed_senders", [])
        }

        gcs_host = str(
            rospy.get_param("~communication/udp/gcs_host", "127.0.0.1")
        )
        gcs_port = int(
            rospy.get_param("~communication/udp/gcs_port", bind_port)
        )
        self.gcs_address = UdpEndpoint("gcs", gcs_host, gcs_port)
        self.u2u_address = self._load_peer_endpoints(
            rospy.get_param("~communication/udp/peers", {})
        )
        self.peer_address_map = {
            endpoint.uav_id: endpoint for endpoint in self.u2u_address
        }

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((bind_host, bind_port))
        self._socket.setblocking(False)
        rospy.loginfo(
            "[SEAD UDP] UAV%d listening on %s:%d; gcs=%s:%d; peers=%s",
            self.uav_id,
            bind_host,
            bind_port,
            gcs_host,
            gcs_port,
            [endpoint.uav_id for endpoint in self.u2u_address],
        )

    def _load_peer_endpoints(self, raw_peers) -> list:
        endpoints = []
        if isinstance(raw_peers, dict):
            entries: Iterable = raw_peers.items()
        elif isinstance(raw_peers, list):
            entries = enumerate(raw_peers)
        else:
            entries = []

        for key, value in entries:
            if not isinstance(value, dict):
                continue
            try:
                peer_id = int(value.get("uav_id", key))
                if peer_id == self.uav_id:
                    continue
                endpoints.append(
                    UdpEndpoint(
                        "uav%d" % peer_id,
                        str(value["host"]),
                        int(value["port"]),
                        uav_id=peer_id,
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                rospy.logwarn("[SEAD UDP] ignoring invalid peer %r: %s", value, exc)
        return endpoints

    def read_data(self, timeout: float = 1e-5):
        del timeout  # The onboard loop owns timing; the UDP socket is non-blocking.
        try:
            payload, sender = self._socket.recvfrom(self.max_datagram_size)
        except (BlockingIOError, socket.timeout):
            return None
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "[SEAD UDP] receive failed: %s", exc)
            return None

        if self.allowed_senders and sender[0] not in self.allowed_senders:
            rospy.logwarn_throttle(
                2.0, "[SEAD UDP] rejected datagram from %s", sender[0]
            )
            return None
        if not payload:
            return None
        return FakePacket(payload)

    def send_data_async(self, address, raw_data: bytes):
        endpoint = self._coerce_endpoint(address)
        if endpoint is None:
            rospy.logwarn_throttle(2.0, "[SEAD UDP] unknown destination: %r", address)
            return
        try:
            self._socket.sendto(bytes(raw_data), endpoint.socket_address)
        except OSError as exc:
            rospy.logwarn_throttle(
                2.0, "[SEAD UDP] send to %s failed: %s", endpoint.name, exc
            )

    def send_data_broadcast(self, raw_data: bytes):
        for endpoint in self.u2u_address:
            self.send_data_async(endpoint, raw_data)

    def _coerce_endpoint(self, address):
        if isinstance(address, UdpEndpoint):
            return address
        try:
            address_id = int(address)
        except (TypeError, ValueError):
            return None
        if address_id == 0:
            return self.gcs_address
        return next(
            (peer for peer in self.u2u_address if peer.uav_id == address_id), None
        )

    def close(self):
        try:
            self._socket.close()
        except OSError:
            pass
