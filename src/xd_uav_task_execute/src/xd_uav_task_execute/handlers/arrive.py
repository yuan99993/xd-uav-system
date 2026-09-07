"""Compatibility action for missions whose work ends at arrival."""

from .base import HandlerResult


class ArriveHandler:
    def execute(self, _goal, _should_stop, feedback):
        feedback("running", 1.0, "arrival action requires no post-arrival work")
        return HandlerResult(True, 0, "worker arrival accepted as task completion")
