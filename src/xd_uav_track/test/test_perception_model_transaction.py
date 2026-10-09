#!/usr/bin/env python3
import unittest
from copy import deepcopy
import rospy
from xd_uav_track.msg import SetDetectorModelCommand, DetectorModelCommit, SetPerceptionModeCommand
from test_perception_modes import manager_fixture, Publisher, target
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from xd_uav_track.msg import TrackStateArray


class ConnectedPublisher(Publisher):
    def get_num_connections(self): return 1


class PerceptionModelTest(unittest.TestCase):
    def fixture(self):
        manager = manager_fixture()
        manager._model_transaction = None
        manager._model_request_pub = ConnectedPublisher()
        manager._model.update(sha256="old", backend="tensorrt_fp16", ready=True)
        return manager

    def command(self, manager, request="m1"):
        return SetDetectorModelCommand(request_id=request, session_id=manager._session_id,
            expected_state_version=manager._version, model_id="train4")

    def test_success_preserves_identity_and_motion_clears_old_class_evidence(self):
        m = self.fixture(); m._manual_target = target(); m._selected_uid = target().target_uid
        m._selected_track_id = 7; m._selected_source="fixed"
        m._latest_targets["fixed"] = [target()]
        command=self.command(m); m._model_command_callback(command)
        self.assertEqual(1, m._model["epoch"]); self.assertEqual(1,m._version)
        m._model_commit_callback(DetectorModelCommit(request_id=command.request_id,
            session_id=m._session_id, success=True, model_id="train4", model_epoch=2,
            source_sha256="new", backend="tensorrt_fp16",class_names=["car","tank"]))
        self.assertEqual(7,m._selected_track_id)
        self.assertFalse(m._manual_target.class_valid)
        self.assertEqual({},m._latest_targets)
        self.assertEqual(2,m._version); self.assertEqual(2,m._model["epoch"])
        self.assertEqual("APPLIED",m._result_pub.messages[-1].stage)
        m._model_command_callback(command)
        self.assertEqual(1,len(m._model_request_pub.messages))
        self.assertEqual("APPLIED",m._result_pub.messages[-1].stage)

    def test_failure_keeps_model_selection_and_version(self):
        m=self.fixture(); m._selected_track_id=7
        before=deepcopy(m._model); command=self.command(m)
        m._model_command_callback(command)
        m._model_commit_callback(DetectorModelCommit(request_id=command.request_id,
            session_id=m._session_id,success=False,error_code="INSUFFICIENT_MEMORY",message="reserve"))
        self.assertEqual(before,m._model); self.assertEqual(7,m._selected_track_id)
        self.assertEqual(1,m._version)
        self.assertEqual("FAILED",m._result_pub.messages[-1].stage)

    def test_inflight_dedupe_and_busy_other_command(self):
        m=self.fixture(); cmd=self.command(m); m._model_command_callback(cmd)
        m._model_command_callback(cmd)
        self.assertEqual(1,len(m._model_request_pub.messages))
        m._mode_callback(SetPerceptionModeCommand(request_id="different",session_id=m._session_id,
            expected_state_version=m._version,mode="track"))
        self.assertEqual("BUSY",m._result_pub.messages[-1].error_code)
        self.assertEqual("fusion",m._mode)

    def test_foreign_completion_cannot_commit(self):
        m=self.fixture(); cmd=self.command(m); m._model_command_callback(cmd)
        m._model_commit_callback(DetectorModelCommit(request_id="wrong",session_id=m._session_id,success=True,model_epoch=99))
        self.assertEqual(1,m._model["epoch"]); self.assertIsNotNone(m._model_transaction)

    def test_future_epoch_tracks_wait_for_verified_runtime_and_restart_bumps_version(self):
        m=self.fixture()
        m._tracks_callback(TrackStateArray(model_epoch=2,image_source="fixed",class_names=["tank"]))
        self.assertEqual(1,m._model["epoch"]);self.assertEqual({},m._latest_headers)
        status=DiagnosticStatus(values=[KeyValue(key=k,value=v) for k,v in {
            "model_epoch":"2","active_model_id":"train4","source_model_sha256":"new",
            "backend":"tensorrt_fp16","state":"READY","class_names":'["tank"]'}.items()])
        m._model_callback(DiagnosticArray(status=[status]))
        self.assertEqual(2,m._model["epoch"]);self.assertEqual(2,m._version)
        self.assertEqual("train4",m._model["id"]);self.assertEqual(["tank"],m._class_names)


if __name__ == "__main__": unittest.main()
