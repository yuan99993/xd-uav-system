#!/usr/bin/env python3
import threading
import time
import unittest

import actionlib
import rospy
import rostest

from follower.msg import FollowerStatus
from follower.srv import ManageControlLease, ManageControlLeaseResponse
from pod_msgs.msg import (GimbalState, MissionState, SelectedTarget,
                          TrackTargetAction, TrackTargetGoal,
                          UavControlExecution, UavState)
from pod_msgs.srv import SelectTarget, SelectTargetResponse
from std_srvs.srv import (SetBool, SetBoolResponse, Trigger, TriggerResponse)


class MissionFollowLifecycleTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._state = None
        self._lease_acquires = 0
        self._lease_releases = 0
        self._gimbal_calls = []
        self._stop_calls = 0
        self._selected_pub = rospy.Publisher('/mission_manager_test/selected',
                                             SelectedTarget, queue_size=1)
        self._gimbal_pub = rospy.Publisher('/mission_manager_test/gimbal',
                                           GimbalState, queue_size=1)
        self._uav_pub = rospy.Publisher('/mission_manager_test/uav', UavState,
                                        queue_size=1)
        self._execution_pub = rospy.Publisher('/mission_manager_test/execution',
                                              UavControlExecution, queue_size=1)
        self._follower_pub = rospy.Publisher('/mission_manager_test/follower_status',
                                             FollowerStatus, queue_size=1)
        rospy.Subscriber('/mission_manager_test/state', MissionState,
                         self._state_callback, queue_size=1)
        self._select_service = rospy.Service('/mission_manager_test/select_target',
                                             SelectTarget, self._select)
        self._gimbal_service = rospy.Service('/mission_manager_test/set_gimbal',
                                             SetBool, self._gimbal)
        self._prepare_service = rospy.Service('/mission_manager_test/prepare',
                                              Trigger, self._prepare)
        self._lease_service = rospy.Service('/mission_manager_test/lease',
                                            ManageControlLease, self._lease)
        self._start_service = rospy.Service('/mission_manager_test/follower_start',
                                            SetBool, self._start)
        self._stop_service = rospy.Service('/mission_manager_test/follower_stop',
                                           SetBool, self._stop)
        self._client = actionlib.SimpleActionClient('/mission_manager_test/track_target',
                                                    TrackTargetAction)

    def _state_callback(self, message):
        with self._lock:
            self._state = message

    def _select(self, request):
        response = SelectTargetResponse()
        response.success = request.start_tracking and request.target_id == 7
        response.message = 'selected' if response.success else 'bad target'
        response.selected_target.target_id = request.target_id
        response.selected_target.selected = response.success
        response.selected_target.tracking_active = response.success
        response.selected_target.control_measurement_ready = response.success
        return response

    def _gimbal(self, request):
        with self._lock:
            self._gimbal_calls.append(request.data)
        return SetBoolResponse(success=True, message='ok')

    @staticmethod
    def _prepare(_):
        return TriggerResponse(success=True, message='ready')

    def _lease(self, request):
        with self._lock:
            if request.acquire:
                self._lease_acquires += 1
            else:
                self._lease_releases += 1
        response = ManageControlLeaseResponse()
        response.success = request.requester == 'pod_mission_manager' and \
            request.output_backend == 'command_only'
        response.message = 'ok'
        response.active_requester = request.requester
        response.active_output_backend = request.output_backend
        return response

    @staticmethod
    def _start(request):
        return SetBoolResponse(success=request.data, message='started')

    def _stop(self, request):
        with self._lock:
            self._stop_calls += 1
        return SetBoolResponse(success=request.data, message='stopped')

    def _publish_inputs(self):
        selected = SelectedTarget()
        selected.header.stamp = rospy.Time.now()
        selected.target_id = 7
        selected.selected = True
        selected.tracking_active = True
        selected.control_measurement_ready = True
        selected.confidence = 0.9
        self._selected_pub.publish(selected)
        gimbal = GimbalState()
        gimbal.header.stamp = selected.header.stamp
        gimbal.connected = True
        gimbal.stabilized = True
        gimbal.attitude_valid = True
        self._gimbal_pub.publish(gimbal)
        uav = UavState()
        uav.header.stamp = selected.header.stamp
        uav.connected = True
        uav.armed = True
        uav.offboard_active = True
        uav.localization_valid = True
        uav.control_backend = 'xd'
        self._uav_pub.publish(uav)
        execution = UavControlExecution()
        execution.header.stamp = selected.header.stamp
        execution.accepted = True
        execution.executing = True
        self._execution_pub.publish(execution)
        follower = FollowerStatus()
        follower.header.stamp = selected.header.stamp
        follower.control_authorized = True
        follower.following_active = True
        self._follower_pub.publish(follower)

    def _wait(self, predicate, timeout=4.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not rospy.is_shutdown():
            self._publish_inputs()
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.03)
        return False

    def test_action_owns_lease_renews_and_releases(self):
        self.assertTrue(self._client.wait_for_server(rospy.Duration(5.0)))
        goal = TrackTargetGoal(target_id=7, enable_gimbal_tracking=True,
                               enable_aircraft_follow=True)
        self._client.send_goal(goal)
        self.assertTrue(self._wait(lambda: self._state is not None and
                                   self._state.state == 'TRACKING' and
                                   self._state.aircraft_command_executing))
        self.assertTrue(self._wait(lambda: self._lease_acquires >= 2))
        self._client.cancel_goal()
        self.assertTrue(self._client.wait_for_result(rospy.Duration(5.0)))
        self.assertTrue(self._wait(lambda: self._lease_releases >= 1 and
                                   self._stop_calls >= 1 and False in self._gimbal_calls))


if __name__ == '__main__':
    rospy.init_node('mission_follow_lifecycle_test')
    rostest.rosrun('pod_mission_manager', 'mission_follow_lifecycle',
                   MissionFollowLifecycleTest)
