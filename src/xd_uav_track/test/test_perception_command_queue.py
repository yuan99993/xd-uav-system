#!/usr/bin/env python3
from copy import deepcopy
from collections import deque
import threading
import time
from types import SimpleNamespace
import unittest
import rospy
from std_msgs.msg import Header
from xd_uav_track.command_queue import CommandQueue
from xd_uav_track.msg import SetPerceptionModeCommand, SelectPerceptionTargetCommand, SetDetectorModelCommand, DetectorModelCommit
from test_perception_modes import manager_fixture, target, frame
from test_perception_model_transaction import ConnectedPublisher


class CommandQueueTest(unittest.TestCase):
    def test_bounded_fifo_dedup_conflict_and_exception_resilience(self):
        entered=threading.Event();release=threading.Event();done=threading.Event();seen=[]
        def execute(kind,command):
            seen.append((kind,command.request_id))
            if command.request_id=="1":entered.set();release.wait(2)
            if command.request_id=="2":raise ValueError("bad request")
            if command.request_id=="3":done.set()
        queue=CommandQueue(execute,3);self.addCleanup(queue.close)
        cmd=SimpleNamespace(request_id="1",mode="detect")
        self.assertEqual("QUEUED",queue.submit("mode",cmd));self.assertTrue(entered.wait(1))
        self.assertEqual("DUPLICATE",queue.submit("mode",cmd))
        self.assertEqual("REQUEST_CONFLICT",queue.submit("model",cmd))
        for request in ("2","3"):self.assertEqual("QUEUED",queue.submit("target_select",SimpleNamespace(request_id=request)))
        self.assertEqual("BUSY",queue.submit("mode",SimpleNamespace(request_id="4")))
        release.set();self.assertTrue(done.wait(2))
        self.assertEqual([("mode","1"),("target_select","2"),("target_select","3")],seen)
        self.assertEqual(1,queue.errors)

    def test_real_model_transaction_blocks_other_kinds_until_completion(self):
        m=manager_fixture();m._model_request_pub=ConnectedPublisher()
        m._model.update(sha256="old",backend="tensorrt_fp16",ready=True)
        m._commands=CommandQueue(m._execute_command);self.addCleanup(m.close)
        model=SetDetectorModelCommand(request_id="m",session_id=m._session_id,expected_state_version=1,model_id="train4")
        mode=SetPerceptionModeCommand(request_id="s",session_id=m._session_id,expected_state_version=1,mode="track")
        m._enqueue_command("model",model)
        deadline=time.monotonic()+2
        while not m._model_transaction and time.monotonic()<deadline:time.sleep(.005)
        self.assertIsNotNone(m._model_transaction)
        m._enqueue_command("mode",mode);time.sleep(.05)
        self.assertEqual("fusion",m._mode)
        m._model_commit_callback(DetectorModelCommit(request_id="m",session_id=m._session_id,success=True,
            model_id="train4",model_epoch=2,class_names=["car","tank"],backend="tensorrt_fp16"))
        deadline=time.monotonic()+2
        while not any(x.request_id=="s" and x.stage=="REJECTED" for x in m._result_pub.messages) and time.monotonic()<deadline:time.sleep(.005)
        result=next(x for x in reversed(m._result_pub.messages) if x.request_id=="s")
        self.assertEqual("STALE_STATE",result.error_code)
        self.assertEqual("fusion",m._mode)

    def test_uid_selection_resolves_alias_without_reassigning_identity(self):
        m=manager_fixture(); stamp=rospy.Time(10)
        m._frames["fixed"].append((1,stamp,frame()))
        m._latest_headers["fixed"]=Header(seq=1,stamp=stamp)
        m._track_history["fixed"]=deque([(stamp,[target(31)])])
        m._identity_aliases[7]=31
        cmd=SelectPerceptionTargetCommand(request_id="select",session_id=m._session_id,
            expected_state_version=1,selection_type="track_id",target_uid=m._session_id+":7",
            track_id=-1,image_source="fixed",tracker_algorithm="template")
        m._selection_callback(cmd)
        self.assertEqual("APPLIED",m._result_pub.messages[-1].stage)
        self.assertEqual(31,m._selected_track_id)

    def test_other_session_uid_and_conflicting_id_rejected(self):
        for uid,identity,error in (("old-session:7",-1,"INVALID_TARGET_UID"),("test-session:7",8,"TARGET_CONFLICT")):
            m=manager_fixture()
            cmd=SelectPerceptionTargetCommand(request_id="select",session_id=m._session_id,
                expected_state_version=1,selection_type="track_id",target_uid=uid,track_id=identity,image_source="fixed")
            m._selection_callback(cmd)
            self.assertEqual(error,m._result_pub.messages[-1].error_code)

    def test_queue_submit_is_low_overhead_and_keeps_no_images(self):
        entered=threading.Event();release=threading.Event()
        def execute(*args):entered.set();release.wait(1)
        q=CommandQueue(execute,16);self.addCleanup(q.close)
        start=time.perf_counter()
        for i in range(1000):q.submit("mode",SimpleNamespace(request_id=str(i),mode="detect"))
        elapsed=time.perf_counter()-start
        self.assertLess(elapsed,.2);self.assertLessEqual(len(q.requests),16)
        release.set()


if __name__=="__main__":unittest.main()
