"""Common handler result used by the action server."""

from dataclasses import dataclass


@dataclass(frozen=True)
class HandlerResult:
    success: bool
    error_code: int
    message: str
    preempted: bool = False
