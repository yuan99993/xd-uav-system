#!/usr/bin/env python3
"""Actual rosbridge reconnect/ROI UID/common queue/real GPU model switch."""
import asyncio
import json
import threading
import time
import unittest
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import Image
from tornado.websocket import websocket_connect


class WebContractTest(unittest.TestCase):
    ROOT="/web_contract_test/perception/"
    def setUp(self):
        if not rospy.core.is_initialized():
            self.skipTest("run with rostest perception_web_contract.test")
        self.stopped=threading.Event();self.images=[]
        self.pubs=[rospy.Publisher("/web_contract_test/"+name+"_camera/image_raw",Image,queue_size=1) for name in ("fixed","gimbal")]
        self.worker=threading.Thread(target=self.frames,daemon=True);self.worker.start();self.addCleanup(self.stop)
    def stop(self):self.stopped.set();self.worker.join(timeout=1)
    def frames(self):
        frame=np.zeros((480,640,3),np.uint8)
        frame[180:244,240:336]=np.random.RandomState(43).randint(20,220,(64,96,3)).astype(np.uint8)
        while not self.stopped.is_set() and not rospy.is_shutdown():
            for pub in self.pubs:
                msg=Image();msg.header.stamp=rospy.Time.now();msg.header.frame_id="camera_optical"
                msg.width=640;msg.height=480;msg.encoding="bgr8";msg.step=1920;msg.data=frame.tobytes();pub.publish(msg)
                self.images.append(msg)
                self.images=self.images[-4:]
            self.stopped.wait(.1)
    async def connect(self):
        deadline=time.monotonic()+20
        while True:
            try:return await websocket_connect("ws://127.0.0.1:24177",connect_timeout=2)
            except Exception:
                if time.monotonic()>deadline:raise
                await asyncio.sleep(.1)
    async def read(self, socket, predicate, seconds=25):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            raw=await asyncio.wait_for(socket.read_message(),max(.01,deadline-time.monotonic()))
            if raw is None:raise RuntimeError("websocket unexpectedly closed")
            packet=json.loads(raw)
            if predicate(packet):return packet
        raise RuntimeError("bounded websocket wait timed out")
    async def service(self,socket,suffix):
        socket.write_message(json.dumps(dict(op="call_service",id=suffix,service=self.ROOT+suffix,args={})))
        packet=await self.read(socket,lambda p:p.get("op")=="service_response" and p.get("id")==suffix)
        self.assertTrue(packet["result"],packet);self.assertTrue(packet["values"]["success"],packet)
        return packet["values"]
    async def setup_socket(self):
        socket=await self.connect()
        for name in ("command_result","state","targets","trajectory"):
            socket.write_message(json.dumps(dict(op="subscribe",topic=self.ROOT+name,queue_length=32 if name=="command_result" else 1)))
        return socket
    def command(self,state,request,**fields):
        return dict(request_id=request,session_id=state["session_id"],expected_state_version=state["state_version"],
            issued_at=state["header"]["stamp"],ttl_sec=20,**fields)
    def send(self,socket,suffix,message):
        types={"mode/set":"SetPerceptionModeCommand","target/select":"SelectPerceptionTargetCommand","model/set":"SetDetectorModelCommand"}
        socket.write_message(json.dumps(dict(op="advertise",topic=self.ROOT+suffix,type="xd_uav_track/"+types[suffix])))
        socket.write_message(json.dumps(dict(op="publish",topic=self.ROOT+suffix,msg=message)))
    async def terminal(self,socket,request):
        packet=await self.read(socket,lambda p:p.get("topic")==self.ROOT+"command_result" and
            p["msg"]["request_id"]==request and p["msg"]["stage"] in ("APPLIED","FAILED","REJECTED"))
        return packet["msg"]
    async def scenario(self):
        rospy.wait_for_service(self.ROOT+"state/get", timeout=30)
        socket=await self.setup_socket()
        try:
            state=(await self.service(socket,"state/get"))["state"]
            while not state["model_ready"]:
                state=(await self.read(socket,lambda p:p.get("topic")==self.ROOT+"state" and p["msg"]["model_ready"],seconds=60))["msg"]
            registry=await self.service(socket,"models/list")
            self.assertEqual({"train","train4","train7"},{m["model_id"] for m in registry["models"]})
            self.assertTrue(all(m["engine_prepared"] for m in registry["models"]))
            self.assertEqual(state["model_epoch"],registry["model_epoch"])
            # Bind ROI to a just-captured original image, not a latest overlap.
            stamp=self.images[-2].header.stamp
            roi=self.command(state,"web-roi",selection_type="roi",image_source="fixed_rgb",track_id=-1,target_uid="",
                frame_seq=0,capture_stamp=dict(secs=stamp.secs,nsecs=stamp.nsecs),point_norm=[0.,0.],
                roi_norm=[240/640,180/480,96/640,64/480],tracker_algorithm="template")
            self.send(socket,"target/select",roi)
            result=await self.terminal(socket,"web-roi");self.assertEqual("APPLIED",result["stage"],result)
            state=(await self.service(socket,"state/get"))["state"];uid=state["selected_target_uid"]
            trajectory=(await self.read(socket,lambda p:p.get("topic")==self.ROOT+"trajectory" and
                p["msg"]["target_uid"]==uid and p["msg"]["points"]))["msg"]
            self.assertLessEqual(len(trajectory["points"]),600)
            # Lose transport after submit. Reconnect and replay unchanged
            # expected version: cached APPLIED must win over stale-state check.
            cmd=self.command(state,"web-fusion",mode="fusion")
            self.send(socket,"mode/set",cmd)
            await asyncio.sleep(.08);socket.close();socket=await self.setup_socket()
            state=(await self.service(socket,"state/get"))["state"]
            self.assertEqual("fusion",state["mode"])
            self.send(socket,"mode/set",cmd)
            result=await self.terminal(socket,"web-fusion");self.assertEqual("APPLIED",result["stage"],result)
            after=(await self.service(socket,"state/get"))["state"]
            self.assertEqual(state["state_version"],after["state_version"])
            self.assertEqual(uid,after["selected_target_uid"])
            # UID-only request must preserve the canonical manual global ID.
            select=dict(roi);select.update(self.command(after,"web-uid"),selection_type="track_id",target_uid=uid,frame_seq=0,capture_stamp=dict(secs=0,nsecs=0))
            self.send(socket,"target/select",select)
            result=await self.terminal(socket,"web-uid");self.assertEqual("APPLIED",result["stage"],result)
            after=(await self.service(socket,"state/get"))["state"]
            switch=self.command(after,"web-model",model_id="train4")
            mode=self.command(after,"web-conflict",mode="detect")
            self.send(socket,"model/set",switch)
            await self.read(socket,lambda p:p.get("topic")==self.ROOT+"command_result" and
                p["msg"]["request_id"]=="web-model" and p["msg"]["stage"]=="ACCEPTED")
            self.send(socket,"mode/set",mode)
            model_result=await self.terminal(socket,"web-model");self.assertEqual("APPLIED",model_result["stage"],model_result)
            conflicting=await self.terminal(socket,"web-conflict")
            self.assertEqual("STALE_STATE",conflicting["error_code"],conflicting)
            final=(await self.service(socket,"state/get"))["state"]
            self.assertEqual("train4",final["active_model_id"]);self.assertEqual(uid,final["selected_target_uid"])
            self.assertEqual("fusion",final["mode"])
            latencies=[]
            for _ in range(5):
                start=time.perf_counter()
                snapshot=await self.service(socket,"state/get")
                latencies.append((time.perf_counter()-start)*1000)
                self.assertEqual(final["state_version"],snapshot["state"]["state_version"])
            p95=float(np.percentile(latencies,95))
            self.assertLess(p95,250.,"read-only state RPC starved by perception")
            rospy.loginfo("WEB_CONTRACT_PASS reconnect, exact replay, ROI/global UID, trajectory, list/get, serial GPU model transaction; state_get_p95_ms=%.3f",p95)
        finally:socket.close()
    def test_actual_websocket_reconnect_and_transactions(self):asyncio.run(self.scenario())


if __name__=="__main__":
    rospy.init_node("test_perception_web_ros")
    rostest.rosrun("xd_uav_track","websocket_contract",WebContractTest)
