#!/usr/bin/env python3

"""Factory for SEAD communication transports."""

from xd_uav_sead.comms.rosbridge import SeadRosBridge
from xd_uav_sead.comms.udp_transport import RawUdpTransport
from xd_uav_sead.comms.xbee_transport import XBeeTransport


def create_transport(mode: str, uav_name: str, uav_id: int):
    normalized = str(mode or "ros").strip().lower()
    if normalized == "ros":
        return SeadRosBridge(uav_name, uav_id)
    if normalized == "udp":
        return RawUdpTransport(uav_id)
    if normalized == "xbee":
        return XBeeTransport(uav_id)
    raise ValueError(
        "unsupported SEAD communication mode %r; expected ros, udp or xbee"
        % mode
    )
