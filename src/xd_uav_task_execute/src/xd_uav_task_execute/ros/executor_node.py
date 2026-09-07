"""Per-aircraft ExecuteTask action server."""

import traceback

import actionlib
import rospy

from xd_uav_task_execute.handlers import ArriveHandler, TrackHandler
from xd_uav_task_execute.handlers.base import HandlerResult
from xd_uav_task_execute.msg import (
    ExecuteTaskAction,
    ExecuteTaskFeedback,
    ExecuteTaskGoal,
    ExecuteTaskResult,
    TaskExecutionStatus,
)


class TaskExecuteNode:
    def __init__(self):
        self.worker_name = str(rospy.get_param("~worker_name", "")).strip()
        if not self.worker_name:
            raise ValueError("~worker_name must not be empty")
        self.feedback_rate_hz = max(
            1.0, float(rospy.get_param("~runtime/feedback_rate_hz", 10.0))
        )
        interfaces = dict(rospy.get_param("~interfaces", {}))
        action_name = str(interfaces.get("action_server", "~execute"))
        status_topic = str(interfaces.get("status", "~status"))
        track_interfaces = dict(interfaces.get("track", {}))
        self.status_publisher = rospy.Publisher(
            status_topic, TaskExecutionStatus, queue_size=5, latch=True
        )
        self.handlers = {
            ExecuteTaskGoal.ARRIVE: ArriveHandler(),
            ExecuteTaskGoal.TRACK: TrackHandler(
                dict(rospy.get_param("~track", {})),
                track_interfaces,
                self.feedback_rate_hz,
            ),
        }
        self.server = actionlib.SimpleActionServer(
            action_name,
            ExecuteTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        self._publish_status(0, 0, 0, TaskExecutionStatus.IDLE, 0.0, "idle")
        rospy.loginfo(
            "[task_execute] worker=%s ready, handlers=arrive,track action=%s",
            self.worker_name,
            rospy.resolve_name(action_name),
        )

    def _publish_status(self, task_id, target_id, task_type, phase, progress, detail):
        status = TaskExecutionStatus()
        status.header.stamp = rospy.Time.now()
        status.task_id = int(task_id)
        status.target_id = int(target_id)
        status.task_type = int(task_type)
        status.phase = int(phase)
        status.worker_name = self.worker_name
        status.progress = float(progress)
        status.detail = str(detail)
        self.status_publisher.publish(status)

    @staticmethod
    def _feedback_phase(name):
        return {
            "starting": ExecuteTaskFeedback.STARTING,
            "acquiring": ExecuteTaskFeedback.ACQUIRING,
            "running": ExecuteTaskFeedback.RUNNING,
            "stopping": ExecuteTaskFeedback.STOPPING,
        }.get(str(name), ExecuteTaskFeedback.IDLE)

    def _execute(self, goal):
        started_at = rospy.Time.now().to_sec()
        result = ExecuteTaskResult()
        if goal.task_id == 0:
            return self._abort(goal, result, result.INVALID_GOAL, "task_id must be non-zero")
        if goal.worker_name and goal.worker_name != self.worker_name:
            return self._abort(
                goal,
                result,
                result.INVALID_GOAL,
                f"goal is assigned to {goal.worker_name}, executor is {self.worker_name}",
            )
        handler = self.handlers.get(int(goal.task_type))
        if handler is None:
            return self._abort(
                goal,
                result,
                result.UNSUPPORTED_TASK,
                f"task type {goal.task_type} is not implemented",
            )

        def feedback(phase_name, progress, detail):
            phase = self._feedback_phase(phase_name)
            message = ExecuteTaskFeedback()
            message.phase = phase
            message.progress = float(progress)
            message.active_time = rospy.Duration(
                max(0.0, rospy.Time.now().to_sec() - started_at)
            )
            message.detail = str(detail)
            self.server.publish_feedback(message)
            self._publish_status(
                goal.task_id,
                goal.target_id,
                goal.task_type,
                phase,
                progress,
                detail,
            )

        try:
            if int(goal.task_type) == ExecuteTaskGoal.TRACK:
                outcome, elapsed = handler.execute(
                    goal,
                    self.server.is_preempt_requested,
                    feedback,
                    ExecuteTaskResult,
                )
            else:
                outcome = handler.execute(
                    goal, self.server.is_preempt_requested, feedback
                )
                elapsed = max(0.0, rospy.Time.now().to_sec() - started_at)
        except Exception as error:  # Keep the action server alive after a handler bug.
            rospy.logerr("[task_execute] handler exception: %s\n%s", error, traceback.format_exc())
            outcome = HandlerResult(False, result.INTERNAL_ERROR, str(error))
            elapsed = max(0.0, rospy.Time.now().to_sec() - started_at)

        result.success = bool(outcome.success)
        result.error_code = int(outcome.error_code)
        result.message = str(outcome.message)
        result.execution_time = rospy.Duration(elapsed)
        if outcome.preempted:
            self._publish_status(
                goal.task_id, goal.target_id, goal.task_type,
                TaskExecutionStatus.PREEMPTED, 0.0, outcome.message,
            )
            self.server.set_preempted(result, outcome.message)
        elif outcome.success:
            self._publish_status(
                goal.task_id, goal.target_id, goal.task_type,
                TaskExecutionStatus.SUCCEEDED, 1.0, outcome.message,
            )
            self.server.set_succeeded(result, outcome.message)
        else:
            self._publish_status(
                goal.task_id, goal.target_id, goal.task_type,
                TaskExecutionStatus.FAILED, 0.0, outcome.message,
            )
            self.server.set_aborted(result, outcome.message)

    def _abort(self, goal, result, code, detail):
        result.success = False
        result.error_code = int(code)
        result.message = str(detail)
        result.execution_time = rospy.Duration(0.0)
        self._publish_status(
            goal.task_id,
            goal.target_id,
            goal.task_type,
            TaskExecutionStatus.FAILED,
            0.0,
            detail,
        )
        self.server.set_aborted(result, detail)


def main():
    rospy.init_node("task_execute")
    TaskExecuteNode()
    rospy.spin()
