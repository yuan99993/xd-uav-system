#!/usr/bin/env python3

import unittest

from xd_uav_sead.comms.udp_transport import RawUdpTransport, UdpEndpoint


class _FakeSocket:
    def __init__(self):
        self.sent = []
        self.received = []
        self.closed = False

    def sendto(self, payload, destination):
        self.sent.append((payload, destination))

    def recvfrom(self, _max_size):
        if not self.received:
            raise BlockingIOError()
        return self.received.pop(0)

    def close(self):
        self.closed = True


class RawUdpTransportTest(unittest.TestCase):
    def setUp(self):
        self.transport = object.__new__(RawUdpTransport)
        self.transport.uav_id = 1
        self.transport.max_datagram_size = 65507
        self.transport.allowed_senders = set()
        self.transport._socket = _FakeSocket()

    def tearDown(self):
        self.transport.close()

    def test_send_keeps_binary_packet_unchanged(self):
        endpoint = UdpEndpoint("gcs", "127.0.0.1", 15660)
        payload = b"\x12\x01\x00\xffSEAD"
        self.transport.send_data_async(endpoint, payload)
        self.assertEqual(
            self.transport._socket.sent,
            [(payload, ("127.0.0.1", 15660))],
        )

    def test_receive_exposes_xbee_compatible_data(self):
        payload = b"\x18\x02raw-packet"
        self.transport._socket.received.append((payload, ("127.0.0.1", 42000)))
        packet = self.transport.read_data()
        self.assertIsNotNone(packet)
        self.assertEqual(packet.data, payload)

    def test_broadcast_sends_once_to_each_configured_peer(self):
        self.transport.u2u_address = [
            UdpEndpoint("uav2", "10.0.0.2", 15662, uav_id=2),
            UdpEndpoint("uav3", "10.0.0.3", 15663, uav_id=3),
        ]
        payload = b"team-state"
        self.transport.send_data_broadcast(payload)
        self.assertEqual(
            self.transport._socket.sent,
            [
                (payload, ("10.0.0.2", 15662)),
                (payload, ("10.0.0.3", 15663)),
            ],
        )


if __name__ == "__main__":
    unittest.main()
