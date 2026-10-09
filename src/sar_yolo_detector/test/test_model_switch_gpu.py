#!/usr/bin/env python3
"""Live CUDA two-source epochs/latency/rollback, with a bounded total runtime."""
import json
import time
import threading
import unittest
from collections import deque
import numpy as np
import rospy
import rostest
from sensor_msgs.msg import Image
from xd_uav_track.msg import DetectionArray, PerceptionState, PerceptionCommandResult, SetDetectorModelCommand


class GpuModelSwitchTest(unittest.TestCase):
    def setUp(self):
        self.state=None; self.results=deque(maxlen=200); self.outputs={name:deque(maxlen=500) for name in ("fixed","gimbal")}
        self.received={name:deque(maxlen=500) for name in self.outputs}
        root="/model_switch_test/"
        self.subs=[rospy.Subscriber(root+"perception/state",PerceptionState,lambda x:setattr(self,"state",x)),
            rospy.Subscriber(root+"perception/command_result",PerceptionCommandResult,self.results.append)]
        self.pubs={}
        for name in self.outputs:
            self.pubs[name]=rospy.Publisher(root+name+"_camera/image_raw",Image,queue_size=1)
            self.subs.append(rospy.Subscriber(root+"detect/input/"+name+"/detections_2d",DetectionArray,
                lambda msg, source=name:self.collect(source,msg),queue_size=20))
        self.model=rospy.Publisher(root+"perception/model/set",SetDetectorModelCommand,queue_size=1)
        self.stopped=threading.Event(); self.worker=threading.Thread(target=self.frames,daemon=True);self.worker.start()
        self.addCleanup(self.stop)

    def stop(self):self.stopped.set();self.worker.join(timeout=1)
    def collect(self,name,msg):
        self.outputs[name].append(msg)
        self.received[name].append((time.monotonic(), (rospy.Time.now()-msg.header.stamp).to_sec()))
    def frames(self):
        frame=np.zeros((480,640,3),np.uint8)
        while not self.stopped.is_set() and not rospy.is_shutdown():
            for name,pub in self.pubs.items():
                msg=Image();msg.header.stamp=rospy.Time.now();msg.header.frame_id=name+"_optical"
                msg.width=640;msg.height=480;msg.encoding="bgr8";msg.step=1920;msg.data=frame.tobytes();pub.publish(msg)
            self.stopped.wait(.25)
    def wait(self,predicate,seconds=20):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:self.fail("bounded wait timed out")
            time.sleep(.02)
    def switch(self,model,index,expected_success=True):
        self.wait(lambda:self.state is not None and self.state.model_ready,30)
        before=self.state.model_epoch
        cmd=SetDetectorModelCommand(request_id="gpu-%d"%index,session_id=self.state.session_id,
            expected_state_version=self.state.state_version,issued_at=rospy.Time.now(),ttl_sec=20,model_id=model)
        started=time.monotonic();self.model.publish(cmd)
        self.wait(lambda:any(x.request_id==cmd.request_id and x.stage in ("APPLIED","FAILED","REJECTED") for x in self.results),25)
        result=next(x for x in reversed(self.results) if x.request_id==cmd.request_id and x.stage!="ACCEPTED")
        if expected_success:
            self.assertEqual("APPLIED",result.stage,result.message)
            self.wait(lambda:self.state.model_epoch==before+1 and self.state.active_model_id==model)
            for name in self.outputs:
                self.wait(lambda:any(x.model_epoch==before+1 for x in self.outputs[name]))
                epochs=[x.model_epoch for x in self.outputs[name]]
                self.assertEqual(sorted(epochs),epochs,"epoch went backwards")
        else:
            self.assertEqual("FAILED",result.stage);self.assertEqual("UNKNOWN_MODEL",result.error_code)
            self.assertEqual(before,self.state.model_epoch)
        return time.monotonic()-started
    def test_live_switch_and_rollback(self):
        # Cold Python/CUDA startup is distinct from live switch latency.
        # Keep it bounded, but don't fail before the engine can become READY.
        self.wait(lambda: self.state is not None and self.state.model_ready and
            all(len(x)>=5 for x in self.outputs.values()),60)
        durations=[]
        for index,model in enumerate(("train4","train","train7"),1):durations.append(self.switch(model,index));time.sleep(1)
        self.switch("not_registered",4,False)
        for messages in self.outputs.values():
            for msg in messages:
                expected=["car","ar-car","tank"] if msg.model_version.startswith("train:") else ["car","ar-car","tank","m142"]
                self.assertEqual(expected,list(msg.class_names))
        report={"switch_seconds":durations,"sources":{}}
        for name,received in self.received.items():
            values=list(received);latencies=[x[1] for x in values]
            gaps=[b[0]-a[0] for a,b in zip(values,values[1:])]
            report["sources"][name]={"frames":len(values),"latency_p95_ms":float(np.percentile(latencies,95)*1000),"max_gap_sec":max(gaps)}
            self.assertLess(np.percentile(latencies,95),.5)
            self.assertLess(max(gaps),2.0)
        rospy.loginfo("GPU_SWITCH_REPORT %s",json.dumps(report,sort_keys=True))


if __name__=="__main__":
    rospy.init_node("test_model_switch_gpu")
    rostest.rosrun("sar_yolo_detector","gpu_model_switch_contract",GpuModelSwitchTest)
