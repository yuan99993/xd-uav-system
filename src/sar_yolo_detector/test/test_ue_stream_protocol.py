import io
import json
from pathlib import Path
import struct
import sys
import threading
import time
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/ue'))
from stream_protocol import encode_packet,receive_packet,LatestValue,ClockAlignment,HEADER
from ue_stream_client import decode_frame


class StreamProtocolTest(unittest.TestCase):
    def test_bgr8_raw_decode_keeps_original_pixel_buffer(self):
        pixels = np.arange(320*240*3, dtype=np.uint8).reshape(240, 320, 3)
        data = pixels.tobytes()
        decoded = decode_frame(dict(width=320, height=240, codec="bgr8"), data)
        np.testing.assert_array_equal(pixels, decoded)
        self.assertTrue(np.shares_memory(decoded, np.frombuffer(data, dtype=np.uint8)))
        with self.assertRaises(ValueError):
            decode_frame(dict(width=320, height=240, codec="bgr8"), data[:-1])

    def test_ros_send_buffer_is_bounded_per_new_peer_without_changing_framing(self):
        import socket
        from types import SimpleNamespace
        from ue_stream_client import bound_ros_send_buffers
        class Wire:
            def __init__(self):self.calls=[]
            def setsockopt(self,*args):self.calls.append(args)
        a,b=Wire(),Wire();connections=[SimpleNamespace(socket=a)]
        publisher=SimpleNamespace(impl=SimpleNamespace(connections=connections));configured=set()
        bound_ros_send_buffers(publisher,0,configured)
        self.assertFalse(a.calls)
        bound_ros_send_buffers(publisher,65536,configured)
        bound_ros_send_buffers(publisher,65536,configured)
        self.assertEqual(a.calls,[(socket.SOL_SOCKET,socket.SO_SNDBUF,65536),(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)])
        connections[:]=[SimpleNamespace(socket=b)]
        bound_ros_send_buffers(publisher,65536,configured)
        self.assertEqual(configured,{b});self.assertEqual(b.calls,a.calls)

    def test_jpeg_passthrough_validates_geometry_without_an_rgb_decode(self):
        from ue_stream_client import jpeg_dimensions
        import cv2
        _,payload=cv2.imencode('.jpg',np.full((24,32,3),90,np.uint8))
        self.assertEqual(jpeg_dimensions(payload.tobytes()),(32,24))
        with self.assertRaises(ValueError):jpeg_dimensions(payload.tobytes()[:-3])
        with self.assertRaises(ValueError):jpeg_dimensions(b'not an image')

    def test_native_frames_do_not_replay_persisted_reload_file_command(self):
        from frame_protocol import claim_request
        state={};reload=dict(session='reload',id=19,op='reload')
        self.assertTrue(claim_request(state,reload))
        for sequence in range(100):
            self.assertTrue(claim_request(state,dict(session='capture',id=sequence),native=True))
            self.assertFalse(claim_request(state,reload))
        self.assertTrue(claim_request(state,dict(session='new_file_request',id=1)))
    def test_feedback_does_not_hide_missing_camera_frames(self):
        from stream_protocol import StreamWatchdog
        watchdog=StreamWatchdog(100.)
        for i in range(1,7):watchdog.received('gimbal',100+i)
        self.assertIn('camera frames stalled',watchdog.reason(106.))
        watchdog.received('frame',106.);self.assertIsNone(watchdog.reason(107.))
        self.assertIn('feedback stalled',watchdog.reason(112.))

    def test_fragmentation_preserves_frame_metadata_and_bytes(self):
        body=bytes(range(256))*50
        packet=encode_packet(dict(kind='frame',frame_seq=17,capture_wall_start=100.,horizontal_fov_deg=90),body)
        stream=io.BytesIO(packet)
        metadata,result=receive_packet(lambda n:stream.read(min(n,7)))
        self.assertEqual(metadata['frame_seq'],17);self.assertEqual(result,body)

    def test_oversize_and_truncated_frames_fail_without_fake_image(self):
        for packet in [HEADER.pack(b'SRF1',65537,0),HEADER.pack(b'SRF1',20,2**31),HEADER.pack(b'BAD!',1,0)]:
            with self.assertRaises(ValueError):receive_packet(io.BytesIO(packet).read)
        with self.assertRaises(EOFError):receive_packet(io.BytesIO(encode_packet(dict(frame_seq=1),b'abcd')[:-1]).read)

    def test_latest_only_drops_completed_old_frame_and_has_one_slot(self):
        slot=LatestValue()
        for seq in range(1000):slot.put(seq,dict(sequence=seq))
        self.assertEqual(slot.replaced,999)
        self.assertEqual(slot.get()['sequence'],999)
        self.assertFalse(slot.put(800,{}))
        self.assertEqual(slot.out_of_order,1)
        slot.close();self.assertFalse(slot.put(1001,{}));self.assertIsNone(slot.get())

    def test_clock_offset_keeps_actual_delay_and_uncertainty(self):
        clock=ClockAlignment();clock.add(100.,100.028,100.038,100.018)
        result=clock.result();self.assertAlmostEqual(result['ue_minus_host_sec'],.024)
        self.assertAlmostEqual(result['uncertainty_sec'],.004)
        with self.assertRaises(ValueError):clock.add(100.,100.1,100.,100.2)

    def test_raw_colour_channel_contract_and_size(self):
        image=np.zeros((240,320,4),np.uint8);image[:]=[12,34,56,255]
        result=decode_frame(dict(width=320,height=240,codec='bgra8'),image.tobytes())
        self.assertEqual(result[0,0].tolist(),[12,34,56])
        result=decode_frame(dict(width=320,height=240,codec='rgba8'),image.tobytes())
        self.assertEqual(result[0,0].tolist(),[56,34,12])
        with self.assertRaises(ValueError):decode_frame(dict(width=320,height=240,codec='bgra8'),b'incomplete')

    def test_ros_camera_contract_keeps_optical_frame_and_capture_stamp(self):
        from ue_stream_client import camera_messages
        meta=dict(frame_seq=17,capture_wall_start=100.,horizontal_fov_deg=90.,
            camera_position_ned_m=[1.,2.,-30.],measured_camera_rotation_deg=dict(pitch=-70,yaw=30,roll=0))
        image=np.zeros((540,960,3),np.uint8)
        msg,info,transform=camera_messages(meta,image,'camera_optical','ue_render_ned')
        self.assertEqual(msg.header.frame_id,'camera_optical')
        self.assertEqual(info.header.frame_id,'camera_optical')
        self.assertEqual(transform.header.frame_id,'ue_render_ned')
        self.assertEqual(transform.child_frame_id,'camera_optical')
        self.assertEqual(msg.header.stamp,info.header.stamp)
        self.assertEqual(msg.header.stamp,transform.header.stamp)
        self.assertEqual(msg.header.seq,17)
        self.assertAlmostEqual(info.K[0],480.)
        transform.header.frame_id='changed'
        self.assertEqual(msg.header.frame_id,'camera_optical')

if __name__=='__main__':unittest.main()
