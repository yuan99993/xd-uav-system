"""Deterministic instance and port allocation for terrain SITL vehicles."""

from __future__ import annotations

from dataclasses import dataclass
import re


_UAV_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class FleetSlot:
    instance: int
    system_id: int
    mavlink_udp_port: int
    mavlink_tcp_port: int
    mavros_bind_port: int
    mavros_remote_port: int
    gst_udp_port: int
    camera_udp_port: int

    @property
    def mavros_fcu_url(self):
        return "udp://:{0}@localhost:{1}".format(
            self.mavros_bind_port, self.mavros_remote_port)


def slot_for_instance(instance):
    """Return the repository's established PX4-SITL port layout."""
    value = int(instance)
    if value < 0 or value >= 255:
        raise ValueError("instance must be in [0, 254]")
    ports = (
        14560 + value,
        4560 + value,
        14540 + value,
        14580 + value,
        5600 + value,
        14530 + value,
    )
    if any(port > 65535 for port in ports):
        raise ValueError("instance produces an invalid network port")
    return FleetSlot(
        instance=value,
        system_id=value + 1,
        mavlink_udp_port=ports[0],
        mavlink_tcp_port=ports[1],
        mavros_bind_port=ports[2],
        mavros_remote_port=ports[3],
        gst_udp_port=ports[4],
        camera_udp_port=ports[5],
    )


class FleetAllocator:
    """Allocate the lowest available SITL slot and reject ambiguous names."""

    def __init__(self, capacity=8):
        self.capacity = int(capacity)
        if self.capacity < 1 or self.capacity > 255:
            raise ValueError("capacity must be in [1, 255]")
        slot_for_instance(self.capacity - 1)
        self._by_name = {}
        self._used = set()

    @staticmethod
    def validate_name(name):
        value = str(name)
        if not _UAV_NAME.fullmatch(value):
            raise ValueError("uav_name must be a lowercase ROS name component")
        return value

    def allocate(self, name, preferred_instance=None):
        value = self.validate_name(name)
        if value in self._by_name:
            raise ValueError("vehicle already exists: " + value)
        if preferred_instance is None:
            available = next(
                (item for item in range(self.capacity) if item not in self._used),
                None)
            if available is None:
                raise ValueError("fleet capacity is full")
            instance = available
        else:
            instance = int(preferred_instance)
            if instance < 0 or instance >= self.capacity:
                raise ValueError("preferred instance is outside fleet capacity")
            if instance in self._used:
                raise ValueError("preferred instance is already allocated")
        slot = slot_for_instance(instance)
        self._by_name[value] = slot
        self._used.add(instance)
        return slot

    def release(self, name):
        value = self.validate_name(name)
        slot = self._by_name.pop(value, None)
        if slot is None:
            raise ValueError("vehicle does not exist: " + value)
        self._used.remove(slot.instance)
        return slot

    def get(self, name):
        return self._by_name.get(str(name))

    def names(self):
        return tuple(sorted(self._by_name))


def validate_removal_state(connected, armed, on_ground):
    """Fail closed unless the autopilot proves the vehicle is safe to remove."""
    if not connected:
        raise ValueError("vehicle state is unavailable")
    if armed:
        raise ValueError("vehicle is armed")
    if not on_ground:
        raise ValueError("vehicle is not confirmed on the ground")
