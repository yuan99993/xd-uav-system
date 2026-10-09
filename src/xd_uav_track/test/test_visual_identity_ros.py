#!/usr/bin/env python3
"""Actual C++/Python ROS contract; synthetic images, no YOLO/PX4/Gazebo."""
from copy import deepcopy
import time
import unittest

import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from xd_uav_track.msg import (DetectionArray, DetectionCandidate, PerceptionState,
    PerceptionTargetArray, PerceptionCommandResult, SelectPerceptionTargetCommand,
    SetPerceptionModeCommand, TrackStateArray, TargetTrajectoryArray)
from xd_uav_track.srv import (ManageVisualIdentity, ManageVisualIdentityRequest,
                             StartTracker, SetProfile, SelectTrack)


class VisualIdentityRosTest(unittest.TestCase):
    def setUp(self):
        if not rospy.core.is_initialized():
            self.skipTest("run with rostest perception_visual_identity.test")
        self.state=None; self.outputs=[]; self.results=[]; self.tracks=[];self.trajectories=[]
        self.bridge=CvBridge()
        self.namespace="/perception_test/"
        self.subscribers=[
            rospy.Subscriber(self.namespace+"perception/trajectory",TargetTrajectoryArray,self.trajectories.append),
            rospy.Subscriber(self.namespace+"perception/state",PerceptionState,
                             lambda msg:setattr(self,"state",msg)),
            rospy.Subscriber(self.namespace+"perception/targets",PerceptionTargetArray,self.outputs.append),
            rospy.Subscriber(self.namespace+"perception/command_result",PerceptionCommandResult,self.results.append),
            rospy.Subscriber(self.namespace+"track/tracks_by_source",TrackStateArray,self.tracks.append)]
        self.images={source:rospy.Publisher(self.namespace+source+"/image_raw",Image,queue_size=1)
                     for source in ("fixed","gimbal")}
        self.detections=rospy.Publisher(self.namespace+"detect/detections",DetectionArray,queue_size=1)
        self.odom=rospy.Publisher(self.namespace+"state_estimator/main/odom",Odometry,queue_size=1)
        self.select=rospy.Publisher(self.namespace+"perception/target/select",SelectPerceptionTargetCommand,queue_size=1)
        self.mode=rospy.Publisher(self.namespace+"perception/mode/set",SetPerceptionModeCommand,queue_size=1)
        rospy.wait_for_service(self.namespace+"track/manage_visual_identity",timeout=8)
        self.identity=rospy.ServiceProxy(self.namespace+"track/manage_visual_identity",ManageVisualIdentity)
        self.await_condition(lambda:self.state is not None and self.detections.get_num_connections()
                             and self.odom.get_num_connections()
                             and all(p.get_num_connections() for p in self.images.values()))
        self.request_counter=0; self.sequence=0

    def await_condition(self, condition, timeout=4):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not condition(): time.sleep(.01)
        self.assertTrue(condition(),"timed out waiting for ROS evidence")

    def publish_frame(self,source="fixed",detect=False,metric=True,shift=0,blank=False,legacy_command=""):
        self.sequence+=1
        stamp=rospy.Time.now()
        image=np.zeros((120,180,3),np.uint8)
        if not blank:
            image[40:64,50+shift:86+shift]=np.random.RandomState(42).randint(20,220,(24,36,3)).astype(np.uint8)
        message=self.bridge.cv2_to_imgmsg(image,"bgr8")
        message.header.seq=self.sequence; message.header.stamp=stamp; message.header.frame_id=source+"_optical"
        self.images[source].publish(message)
        # Collect before sending a detector result; real inference has the same ordering.
        time.sleep(.04)
        if detect:
            odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id="world"
            odom.pose.pose.orientation.w=1
            self.odom.publish(odom); time.sleep(.02)
            candidate=DetectionCandidate()
            candidate.track_id=-1; candidate.class_id=2; candidate.confidence=.95
            candidate.has_bbox=True; candidate.bbox=[50+shift,40,86+shift,64]
            candidate.has_normalized_bbox=True
            candidate.normalized_bbox=[(68+shift)/180,52/120,36/180,24/120]
            candidate.appearance_embedding=[1.,0.,0.,0.]
            if metric:
                candidate.has_relative_position_body=True; candidate.range_valid=True
                candidate.relative_position_body=[10.,2.,-5.]
                candidate.position_covariance=[.25,0,0,0,.25,0,0,0,.25]
            array=DetectionArray(); array.header=message.header
            array.image_source=source; array.image_width=180;array.image_height=120
            array.command=legacy_command
            array.candidates=[candidate]
            self.detections.publish(array)
            self.await_condition(lambda:any(t.image_source==source and t.header.stamp==stamp for t in self.tracks))
        time.sleep(.05)
        return message

    def command(self,mode=None,selection=None):
        self.await_condition(lambda:self.state is not None)
        self.request_counter+=1
        cmd=SetPerceptionModeCommand() if mode else SelectPerceptionTargetCommand()
        cmd.request_id="contract-{}".format(self.request_counter)
        cmd.session_id=self.state.session_id;cmd.expected_state_version=self.state.state_version
        cmd.issued_at=rospy.Time.now();cmd.ttl_sec=3
        if mode:cmd.mode=mode;self.mode.publish(cmd)
        else:
            for key,value in selection.items():setattr(cmd,key,value)
            self.select.publish(cmd)
        self.await_condition(lambda:any(r.request_id==cmd.request_id and r.stage in ("APPLIED","REJECTED","FAILED") for r in self.results))
        result=next(r for r in reversed(self.results) if r.request_id==cmd.request_id and r.stage!="ACCEPTED")
        if result.stage=="APPLIED":self.await_condition(lambda:self.state.state_version>=result.state_version)
        return result

    def test_registered_roi_binds_preserves_id_and_uses_cross_source_gates(self):
        # Real binary, not a source-text assertion: contradictory configuration
        # and every legacy flight enable path must leave visual identity usable.
        start = rospy.ServiceProxy(self.namespace+"track/start_tracker", StartTracker)
        self.assertFalse(start(True).success)
        self.assertFalse(start(False).active)
        profile = rospy.ServiceProxy(self.namespace+"track/set_profile", SetProfile)
        self.assertFalse(profile("fw_metric_orbit").success)
        flight_select = rospy.ServiceProxy(self.namespace+"track/select_track", SelectTrack)
        self.assertFalse(flight_select(target_id=1,start_tracking=True).success)
        self.assert_no_flight_publishers()
        image=self.publish_frame()
        # Read actual image seq from received message: ROS publisher assigns header.seq.
        selected=self.command(selection={"selection_type":"roi","image_source":"fixed",
            "capture_stamp":image.header.stamp,"roi_norm":[50/180,40/120,36/180,24/120],
            "tracker_algorithm":"template"})
        self.assertEqual("APPLIED",selected.stage,selected.message)
        registered_id=self.state.selected_track_id
        self.assertGreater(registered_id,0);self.assertLess(registered_id,1000000000)
        self.assertEqual("APPLIED",self.command(mode="fusion").stage)
        for index in range(9):self.publish_frame(detect=True,shift=min(index,3),
            legacy_command="start_track" if index == 0 else "")
        self.assert_no_flight_publishers()
        latest=next(t for t in reversed(self.tracks) if t.image_source=="fixed")
        self.assertEqual(registered_id,latest.tracks[0].track_id,"YOLO local identity must inherit registered public ID")
        metric_stamp = latest.header.stamp
        self.await_condition(lambda:any(output.header.stamp == metric_stamp and
            any(item.track_id == registered_id and item.position_valid and
                item.position_frame == "perception_test/base_link_frd" and
                item.position_stamp == metric_stamp for item in output.targets) for output in self.outputs))
        self.assertEqual("APPLIED",self.command(mode="detect").stage)
        image=self.publish_frame(shift=3)
        self.await_condition(lambda:any(m.header.stamp==image.header.stamp and
            any(t.track_id==registered_id and t.lifecycle_state=="confirmed" for t in m.targets) for m in self.outputs))
        visual = next(output for output in reversed(self.outputs) if output.header.stamp == image.header.stamp)
        self.assertTrue(all(not item.position_valid for item in visual.targets),
                        "SOT must not reuse the last FRD localization as a new 3-D observation")
        for output in self.outputs:
            ids=[t.track_id for t in output.targets]
            self.assertEqual(len(ids),len(set(ids)),"one output cannot contain duplicate global IDs")
        # A persisted visual identity is selectable on its original frame in detect.
        result = self.command(selection={"selection_type":"point", "image_source":"fixed",
            "capture_stamp":image.header.stamp, "point_norm":[71/180,52/120],
            "tracker_algorithm":"template"})
        self.assertEqual("APPLIED", result.stage, result.message)
        self.assertEqual(registered_id, self.state.selected_track_id)
        unauthorized = ManageVisualIdentityRequest(action="update",owner_session="another-owner",
            image_source="fixed",global_id=registered_id)
        unauthorized.observation.header.stamp = image.header.stamp
        unauthorized.observation.bbox = [53,40,89,64]
        self.assertFalse(self.identity(unauthorized).success)
        unauthorized.owner_session = self.state.session_id
        self.assertFalse(self.identity(unauthorized).success, "repeated capture cannot refresh identity")
        # A frame no longer in cache must never select an overlapping current box.
        result=self.command(selection={"selection_type":"roi","image_source":"fixed",
            "frame_seq":999999,"capture_stamp":image.header.stamp,
            "roi_norm":[50/180,40/120,36/180,24/120],"tracker_algorithm":"template"})
        self.assertEqual("STALE_FRAME",result.error_code)
        for _ in range(8):
            self.publish_frame("fixed",detect=True,shift=3)
            self.publish_frame("gimbal",detect=True,shift=3)
        gimbal=next(t for t in reversed(self.tracks) if t.image_source=="gimbal")
        self.assertEqual(registered_id,gimbal.tracks[0].track_id,"calibrated cross-source gates must share the global identity")
        # Cross-camera continuity must not be inferred from pixel/appearance alone.
        for _ in range(4):
            stamp = rospy.Time.now()
            candidate = DetectionCandidate(track_id=-1,class_id=2,confidence=.95,
                has_bbox=True,bbox=[53,40,89,64],has_normalized_bbox=True,
                normalized_bbox=[71/180,52/120,36/180,24/120],
                appearance_embedding=[1.,0.,0.,0.])
            array = DetectionArray(image_source="unlocated",image_width=180,image_height=120,
                                   candidates=[candidate])
            array.header.stamp = stamp
            self.detections.publish(array)
            self.await_condition(lambda:any(t.image_source=="unlocated" and
                t.header.stamp==stamp for t in self.tracks))
            time.sleep(.05)
        unlocated=next(t for t in reversed(self.tracks) if t.image_source=="unlocated")
        self.assertTrue(unlocated.tracks)
        self.assertNotEqual(registered_id,unlocated.tracks[0].track_id,
                            "same appearance without world evidence must not cross-source merge")
        vanished = self.publish_frame("fixed", blank=True)
        self.await_condition(lambda: any(output.header.stamp == vanished.header.stamp and
            any(item.track_id == registered_id and item.lifecycle_state == "lost"
                and item.measurement_source == "stale" and list(item.bbox_xyxy) == [0,0,0,0]
                and not item.position_valid for item in output.targets) for output in self.outputs))
        self.await_condition(lambda: self.state.operating_state == "LOST")
        self.await_condition(lambda: any(path.target_uid == self.state.selected_target_uid and
            path.status == "LOST" and path.points for path in self.trajectories))
        self.assertTrue(all(len(path.points)<=600 for path in self.trajectories))
        # Full source capacity and the allowlist both fail closed, never allocate
        # a fourth manager or synthesize a public identity.
        for source in ("overflow", "unknown"):
            array=DetectionArray(image_source=source,image_width=180,image_height=120)
            array.header.stamp=rospy.Time.now()
            self.detections.publish(array)
            time.sleep(.15)
            self.assertFalse(any(frame.image_source==source for frame in self.tracks))
        self.assert_coarse_roi_reuses_existing_identity()
        self.assert_existing_identity_attachment()

    def assert_coarse_roi_reuses_existing_identity(self):
        self.assertEqual('APPLIED',self.command(mode='fusion').stage)
        for _ in range(3): frame=self.publish_frame('gimbal',detect=True,shift=3)
        self.await_condition(lambda:any(o.image_source=='gimbal' and o.header.stamp==frame.header.stamp
                                       and o.targets for o in self.outputs))
        snapshot=next(o for o in reversed(self.outputs) if o.image_source=='gimbal' and
                      o.header.stamp==frame.header.stamp and o.targets)
        candidate=next(t for t in snapshot.targets if t.lifecycle_state=='confirmed' and t.class_valid)
        cx,cy,bw,bh=candidate.bbox_cxcywh_norm
        # Nine times the object area: previously IoU<.25 allocated a new
        # artificial target, trained CSRT on background and produced two IDs.
        result=self.command(selection=dict(selection_type='roi',image_source='gimbal',
            frame_seq=snapshot.frame_seq,capture_stamp=frame.header.stamp,
            roi_norm=[cx-1.5*bw,cy-1.5*bh,3*bw,3*bh],tracker_algorithm='template'))
        self.assertEqual('APPLIED',result.stage,result.message)
        self.assertEqual(candidate.track_id,self.state.selected_track_id)
        self.assertEqual(candidate.target_uid,self.state.selected_target_uid)
        next_frame=self.publish_frame('gimbal',detect=True,shift=3)
        self.await_condition(lambda:any(o.image_source=='gimbal' and
            o.header.stamp==next_frame.header.stamp and any(t.selected for t in o.targets)
            for o in self.outputs))
        output=next(o for o in reversed(self.outputs) if o.image_source=='gimbal' and
                    o.header.stamp==next_frame.header.stamp and any(t.selected for t in o.targets))
        self.assertEqual(1,sum(t.track_id==candidate.track_id for t in output.targets))
        selected=next(t for t in output.targets if t.selected)
        self.assertLess(selected.bbox_cxcywh_norm[2]*selected.bbox_cxcywh_norm[3], 2*bw*bh)

    def assert_existing_identity_attachment(self):
        """Real C++ bind path: ordinary selection, fork, simultaneous vehicles."""
        source = 'unlocated'
        owner = 'selected-observer-contract'
        signature = [1., 0., 0., 0.]
        def publish(reset=False, second=False, distance=None, delayed_pose=False, confidence=.95):
            stamp = rospy.Time.now()
            def candidate(x):
                result = DetectionCandidate(track_id=-1, class_id=2, confidence=confidence,
                    has_bbox=True, bbox=[x,40,x+36,64], has_normalized_bbox=True,
                    normalized_bbox=[(x+18)/180,52/120,36/180,24/120],
                    appearance_embedding=[1.,0.,0.,0.])
                if distance is not None:
                    result.has_relative_position_body=True; result.range_valid=True
                    result.relative_position_body=[distance,2.,-5.]
                    result.position_covariance=[.25,0.,0.,0.,.25,0.,0.,0.,.25]
                return result
            array = DetectionArray(image_source=source, image_width=180, image_height=120,
                                   candidates=[candidate(50)])
            if second: array.candidates.append(candidate(120))
            array.header.stamp = stamp
            if distance is not None:
                odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='world'
                odom.pose.pose.orientation.w=1.
                self.odom.publish(odom); time.sleep(.02)
                if delayed_pose:
                    # Detection is captured before the vehicle rotates/moves.
                    # The latest pose is deliberately very different: global
                    # identity must use the capture pose, not this one.
                    latest=deepcopy(odom); latest.header.stamp=rospy.Time.now()
                    latest.pose.pose.position.x=60. if reset else -60.
                    latest.pose.pose.orientation.w=0.
                    latest.pose.pose.orientation.z=1.
                    self.odom.publish(latest); time.sleep(.02)
            if reset: array.command = 'reset'
            self.detections.publish(array)
            self.await_condition(lambda: any(t.image_source==source and t.header.stamp==stamp
                                            for t in self.tracks))
            return next(t for t in reversed(self.tracks) if t.image_source==source and t.header.stamp==stamp)
        anchor = publish()
        old = next(t for t in anchor.tracks if t.detected and t.lifecycle_state=='confirmed')
        request = ManageVisualIdentityRequest(action='attach', owner_session=owner,
            image_source=source, global_id=old.track_id, observation=deepcopy(old),
            image_signature=signature)
        request.observation.header = anchor.header
        result = self.identity(request)
        self.assertTrue(result.success, result.message)
        self.assertEqual(old.track_id, result.global_id)
        forged = deepcopy(request); forged.owner_session = 'different-owner'
        self.assertFalse(self.identity(forged).success)
        # Retirement is an owner-only metadata operation; it may refer to a
        # lost/zero-area ROI without pretending to submit a new image sample.
        retire=ManageVisualIdentityRequest(action='retire',owner_session='different-owner',
            image_source=source,global_id=old.track_id)
        self.assertFalse(self.identity(retire).success)
        retire.owner_session=owner
        retired=self.identity(retire)
        self.assertTrue(retired.success,retired.message)
        self.assertEqual(old.track_id,retired.global_id)
        replacement = None
        binds = []
        for index in range(5):
            snapshot = publish(reset=index==0)
            candidates = [t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed']
            if not candidates: continue
            replacement = candidates[0].track_id
            self.assertNotEqual(old.track_id, replacement)
            observation = deepcopy(candidates[0]); observation.header = snapshot.header
            observation.track_id = old.track_id
            update = ManageVisualIdentityRequest(action='update', owner_session=owner,
                image_source=source, global_id=old.track_id, observation=observation)
            self.assertTrue(self.identity(update).success)
            update.action='bind'; update.candidate_global_id=replacement
            update.image_signature=signature
            result = self.identity(update); self.assertTrue(result.success, result.message)
            binds.append(result.bound)
            if not result.bound:
                duplicate=self.identity(update)
                self.assertFalse(duplicate.success)
                self.assertEqual('duplicate bind timestamp',duplicate.message)
            if result.bound: break
        self.assertEqual([False,False,True], binds)
        self.await_condition(lambda:any(t.image_source==source and
            any(v.track_id==old.track_id for v in t.tracks) for t in self.tracks[-3:]))
        # Weak current YOLO evidence must not inherit a high smoothed score.
        weak=publish(confidence=.2)
        measured=next(t for t in weak.tracks if t.detected and t.track_id==old.track_id)
        self.assertGreater(measured.confidence,.5)
        proof=ManageVisualIdentityRequest(action='update',owner_session=owner,image_source=source,
            global_id=old.track_id,observation=deepcopy(measured),image_signature=signature)
        proof.observation.header=weak.header
        self.assertTrue(self.identity(proof).success)
        proof.action='bind'; proof.candidate_global_id=old.track_id
        rejected=self.identity(proof)
        self.assertFalse(rejected.success)
        self.assertEqual('ambiguous or inconsistent detector geometry',rejected.message)
        # Conversely, after low-confidence maintenance a genuine strong YOLO
        # frame qualifies immediately, even while the display EMA is < 0.50.
        for _ in range(8): publish(confidence=.2)
        strong=publish()
        measured=next(t for t in strong.tracks if t.detected and t.track_id==old.track_id)
        self.assertLess(measured.confidence,.5)
        proof.action='update'; proof.observation=deepcopy(measured)
        proof.observation.header=strong.header
        self.assertTrue(self.identity(proof).success)
        proof.action='bind'
        qualified=self.identity(proof)
        self.assertTrue(qualified.success,qualified.message)
        self.assertFalse(qualified.bound,'three distinct confirmations are still mandatory')
        # A nearby visual observer cannot collapse a second simultaneously
        # measured vehicle into the original public entity.
        for _ in range(3): snapshot = publish(second=True)
        competitor = next(t for t in snapshot.tracks if t.detected and
                          t.lifecycle_state=='confirmed' and t.track_id!=old.track_id)
        observation = deepcopy(competitor); observation.header = snapshot.header
        observation.track_id=old.track_id
        update = ManageVisualIdentityRequest(action='update', owner_session=owner,
            image_source=source, global_id=old.track_id, observation=observation)
        self.assertTrue(self.identity(update).success)
        update.action='bind'; update.candidate_global_id=competitor.track_id
        update.image_signature=signature
        rejected=self.identity(update)
        self.assertFalse(rejected.success)
        self.assertEqual('original entity is simultaneously visible', rejected.message)
        publish(reset=True)
        time.sleep(1.6)
        for _ in range(3): snapshot=publish()
        replacement=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
        update.observation=deepcopy(replacement); update.observation.header=snapshot.header
        update.observation.track_id=old.track_id; update.action='update'
        self.assertTrue(self.identity(update).success)
        update.action='bind'; update.candidate_global_id=replacement.track_id
        rejected=self.identity(update)
        self.assertFalse(rejected.success)
        self.assertEqual('metric evidence required after long interruption', rejected.message)
        update.action='detach'
        self.assertTrue(self.identity(update).success)
        self.assertFalse(self.identity(update).success)
        # A valid first rebind near the weak-image deadline must not be
        # discarded mid-confirmation while the SAME detector remains visible.
        publish(reset=True)
        for _ in range(3): snapshot=publish()
        anchor=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
        request.global_id=anchor.track_id; request.observation=deepcopy(anchor)
        request.observation.header=snapshot.header
        self.assertTrue(self.identity(request).success)
        initial_stamp=snapshot.header.stamp
        publish(reset=True)
        for _ in range(3): snapshot=publish()
        time.sleep(max(0.,1.1-(rospy.Time.now()-initial_stamp).to_sec()))
        confirmation=[]
        for index in range(3):
            if index: time.sleep(.3)
            snapshot=publish()
            replacement=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
            update.global_id=anchor.track_id; update.observation=deepcopy(replacement)
            update.observation.header=snapshot.header; update.observation.track_id=anchor.track_id
            update.action='update'; self.assertTrue(self.identity(update).success)
            update.action='bind'; update.candidate_global_id=replacement.track_id
            result=self.identity(update)
            self.assertTrue(result.success,result.message)
            confirmation.append(result.bound)
        self.assertGreater((snapshot.header.stamp-initial_stamp).to_sec(),1.5)
        self.assertEqual([False,False,True],confirmation)
        update.action='detach'; self.assertTrue(self.identity(update).success)
        # Fresh metric evidence cannot approve a physically impossible jump.
        publish(reset=True,distance=10.)
        for _ in range(3): snapshot=publish(distance=10.)
        anchor=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
        request.global_id=anchor.track_id; request.observation=deepcopy(anchor)
        request.observation.header=snapshot.header
        self.assertTrue(self.identity(request).success)
        publish(reset=True,distance=100.)
        for _ in range(3): snapshot=publish(distance=100.)
        replacement=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
        self.assertNotEqual(anchor.track_id,replacement.track_id)
        update.global_id=anchor.track_id
        update.observation=deepcopy(replacement); update.observation.header=snapshot.header
        update.observation.track_id=anchor.track_id; update.action='update'
        self.assertTrue(self.identity(update).success)
        update.action='bind'; update.candidate_global_id=replacement.track_id
        rejected=self.identity(update)
        self.assertFalse(rejected.success)
        self.assertEqual('world innovation conflict',rejected.message)
        update.action='detach'; self.assertTrue(self.identity(update).success)
        # A delayed result with the SAME real world point remains bindable,
        # even though the newest vehicle pose is far from its capture pose.
        publish(reset=True,distance=10.,delayed_pose=True)
        for _ in range(3): snapshot=publish(distance=10.,delayed_pose=True)
        anchor=next(t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed')
        request.global_id=anchor.track_id; request.observation=deepcopy(anchor)
        request.observation.header=snapshot.header
        self.assertTrue(self.identity(request).success)
        binds=[]
        for index in range(8):
            snapshot=publish(reset=index==0,distance=10.,delayed_pose=True)
            candidates=[t for t in snapshot.tracks if t.detected and t.lifecycle_state=='confirmed']
            if not candidates: continue
            replacement=candidates[0]
            update.global_id=anchor.track_id; update.observation=deepcopy(replacement)
            update.observation.header=snapshot.header; update.observation.track_id=anchor.track_id
            update.action='update'; self.assertTrue(self.identity(update).success)
            update.action='bind'; update.candidate_global_id=replacement.track_id
            result=self.identity(update); self.assertTrue(result.success,result.message)
            binds.append(result.bound)
            if result.bound: break
        self.assertEqual([False,False,True],binds)
        update.action='detach'; self.assertTrue(self.identity(update).success)

    def assert_no_flight_publishers(self):
        publishers=rospy.get_master().getSystemState()[2][0]
        for topic,nodes in publishers:
            if topic in (self.namespace+"track/velocity_body",self.namespace+"track/command",
                         self.namespace+"control/reference/setpoint"):
                self.assertNotIn(self.namespace+"track",nodes,
                    "pure perception must not even advertise flight-command outputs")


if __name__=="__main__":
    rospy.init_node("visual_identity_contract_test")
    rostest.rosrun("xd_uav_track","visual_identity_contract",VisualIdentityRosTest)
