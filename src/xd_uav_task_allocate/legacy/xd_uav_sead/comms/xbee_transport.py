#!/usr/bin/env python3

"""Hardware XBee transport for the original SEAD binary protocol."""

import fcntl
import glob
import os
import time

import rospy


class XBeeTransport:
    def __init__(self, uav_id: int):
        try:
            from digi.xbee.devices import (
                DigiMeshDevice,
                RemoteDigiMeshDevice,
                XBee64BitAddress,
            )
            from digi.xbee.exception import TimeoutException
        except ImportError as exc:
            raise RuntimeError(
                "digi.xbee is required when communication/mode=xbee"
            ) from exc

        self.uav_id = int(uav_id)
        self._timeout_exception = TimeoutException
        baud = int(rospy.get_param("~communication/xbee/baud", 57600))
        scan_interval = float(
            rospy.get_param("~communication/xbee/scan_interval", 2.0)
        )
        self._device, self._lock_file = self._find_device(
            DigiMeshDevice, baud, scan_interval
        )

        default_devices = {
            "1": "0013A2004126C97C",
            "2": "0013A2004154C5D3",
            "3": "0013A2004105EB5A",
        }
        configured = rospy.get_param(
            "~communication/xbee/uav_addresses", default_devices
        )
        self.u2u_address = []
        self.peer_address_map = {}
        for peer_id, address in sorted(configured.items(), key=lambda item: int(item[0])):
            if int(peer_id) == self.uav_id:
                continue
            remote = RemoteDigiMeshDevice(
                self._device,
                XBee64BitAddress.from_hex_string(str(address)),
            )
            self.peer_address_map[int(peer_id)] = remote
            self.u2u_address.append(remote)
        gcs_hex = str(
            rospy.get_param(
                "~communication/xbee/gcs_address", "0013A2004105EB61"
            )
        )
        self.gcs_address = RemoteDigiMeshDevice(
            self._device, XBee64BitAddress.from_hex_string(gcs_hex)
        )

    def _find_device(self, device_type, baud, scan_interval):
        while not rospy.is_shutdown():
            ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
            for port in ports:
                lock_path = "/tmp/xd_uav_sead_xbee_%s.lock" % os.path.basename(port)
                lock_file = open(lock_path, "w")
                try:
                    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    lock_file.close()
                    continue

                device = device_type(port, baud)
                try:
                    device.open(force_settings=True)
                    if str(device.get_node_id()) == str(self.uav_id):
                        rospy.loginfo("[SEAD XBee] UAV%d using %s", self.uav_id, port)
                        return device, lock_file
                except Exception as exc:
                    rospy.logdebug("[SEAD XBee] probe %s failed: %s", port, exc)
                try:
                    if device.is_open():
                        device.close()
                except Exception:
                    pass
                fcntl.flock(lock_file, fcntl.LOCK_UN)
                lock_file.close()
            rospy.logwarn_throttle(
                10.0, "[SEAD XBee] waiting for node ID %d", self.uav_id
            )
            time.sleep(max(scan_interval, 0.1))
        raise RuntimeError("ROS shutdown while waiting for XBee")

    def read_data(self, timeout: float = 1e-5):
        try:
            return self._device.read_data(timeout=timeout)
        except self._timeout_exception:
            return None

    def send_data_async(self, address, raw_data: bytes):
        self._device.send_data_async(address, raw_data)

    def send_data_broadcast(self, raw_data: bytes):
        self._device.send_data_broadcast(raw_data)

    def close(self):
        try:
            if self._device.is_open():
                self._device.close()
        finally:
            try:
                fcntl.flock(self._lock_file, fcntl.LOCK_UN)
                self._lock_file.close()
            except Exception:
                pass
