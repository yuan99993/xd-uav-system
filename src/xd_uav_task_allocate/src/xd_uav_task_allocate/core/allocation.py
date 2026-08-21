"""Event-driven worker allocation with assignment locking and task queues."""

from dataclasses import dataclass
from math import hypot
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .registry import GlobalTargetRecord, TARGET_CONFIRMED


TASK_PENDING = 0
TASK_ASSIGNED = 1
TASK_EXECUTING = 2
TASK_COMPLETED = 3
TASK_FAILED = 4


@dataclass
class WorkerRecord:
    name: str
    position: List[float]
    vehicle_type: str = "multirotor"
    online: bool = False
    last_update: float = 0.0
    assigned_task: Optional[int] = None


@dataclass
class RescueTaskRecord:
    task_id: int
    target_id: int
    class_id: int
    target_position: List[float]
    goal: List[float]
    priority: int = 0
    allowed_vehicle_types: Tuple[str, ...] = ()
    assigned_worker: str = ""
    status: int = TASK_PENDING
    detail: str = "waiting for an available worker"


class RescueTaskAllocator:
    def __init__(self):
        self.workers: Dict[str, WorkerRecord] = {}
        self.tasks: Dict[int, RescueTaskRecord] = {}
        self._target_to_task: Dict[int, int] = {}
        self._next_task_id = 1

    def register_worker(self, name: str, vehicle_type: str = "multirotor") -> None:
        normalized_type = str(vehicle_type).strip().lower()
        if normalized_type not in ("multirotor", "fixedwing"):
            raise ValueError("vehicle_type must be multirotor or fixedwing")
        worker = self.workers.setdefault(
            str(name),
            WorkerRecord(str(name), [0.0, 0.0, 0.0], normalized_type),
        )
        if worker.vehicle_type != normalized_type:
            raise ValueError(
                f"worker {name} is already registered as {worker.vehicle_type}"
            )

    def update_worker(
        self,
        name: str,
        position: Sequence[float],
        stamp: float,
        online: bool = True,
        vehicle_type: Optional[str] = None,
    ) -> None:
        existing = self.workers.get(str(name))
        resolved_type = (
            vehicle_type
            if vehicle_type is not None
            else existing.vehicle_type if existing is not None else "multirotor"
        )
        self.register_worker(name, resolved_type)
        worker = self.workers[str(name)]
        worker.position = [float(position[0]), float(position[1]), float(position[2])]
        worker.last_update = float(stamp)
        worker.online = bool(online)

    def ensure_task(
        self,
        target: GlobalTargetRecord,
        priority: int = 0,
        allowed_vehicle_types: Sequence[str] = (),
    ) -> RescueTaskRecord:
        allowed = tuple(sorted({str(item).strip().lower() for item in allowed_vehicle_types}))
        if any(item not in ("multirotor", "fixedwing") for item in allowed):
            raise ValueError("allowed vehicle types must be multirotor or fixedwing")
        existing = self._target_to_task.get(int(target.target_id))
        if existing is not None:
            task = self.tasks[existing]
            if task.status in (TASK_PENDING, TASK_ASSIGNED):
                task.target_position = list(target.position)
                if task.status == TASK_PENDING:
                    task.goal = list(target.position)
            return task
        if target.status != TARGET_CONFIRMED:
            raise ValueError("a rescue task can only be created for a confirmed target")
        task = RescueTaskRecord(
            task_id=self._next_task_id,
            target_id=target.target_id,
            class_id=target.class_id,
            target_position=list(target.position),
            goal=list(target.position),
            priority=int(priority),
            allowed_vehicle_types=allowed,
        )
        self.tasks[task.task_id] = task
        self._target_to_task[target.target_id] = task.task_id
        self._next_task_id += 1
        return task

    def assign_pending(self) -> List[Tuple[RescueTaskRecord, WorkerRecord]]:
        assignments: List[Tuple[RescueTaskRecord, WorkerRecord]] = []
        pending = sorted(
            (task for task in self.tasks.values() if task.status == TASK_PENDING),
            key=lambda task: (-task.priority, task.task_id),
        )
        for task in pending:
            available = [
                worker
                for worker in self.workers.values()
                if worker.online and worker.assigned_task is None
                and (
                    not task.allowed_vehicle_types
                    or worker.vehicle_type in task.allowed_vehicle_types
                )
            ]
            if not available:
                # A later task may allow a different vehicle type even when
                # this task currently has no compatible worker.
                continue
            worker = min(
                available,
                key=lambda item: (
                    hypot(
                        item.position[0] - task.target_position[0],
                        item.position[1] - task.target_position[1],
                    ),
                    item.name,
                ),
            )
            task.assigned_worker = worker.name
            task.status = TASK_ASSIGNED
            task.detail = "assigned; waiting for planner execution"
            worker.assigned_task = task.task_id
            assignments.append((task, worker))
        return assignments

    def mark_executing(self, task_id: int) -> bool:
        task = self.tasks.get(int(task_id))
        if task is None or task.status != TASK_ASSIGNED:
            return False
        task.status = TASK_EXECUTING
        task.detail = "planner trajectory active"
        return True

    def complete(self, task_id: int) -> Optional[RescueTaskRecord]:
        task = self.tasks.get(int(task_id))
        if task is None or task.status not in (TASK_ASSIGNED, TASK_EXECUTING):
            return None
        task.status = TASK_COMPLETED
        task.detail = "worker reached the rescue task goal"
        worker = self.workers.get(task.assigned_worker)
        if worker is not None and worker.assigned_task == task.task_id:
            worker.assigned_task = None
        return task

    def release_worker(self, worker_name: str, detail: str) -> Optional[RescueTaskRecord]:
        worker = self.workers.get(str(worker_name))
        if worker is None or worker.assigned_task is None:
            return None
        task = self.tasks.get(worker.assigned_task)
        worker.assigned_task = None
        if task is None or task.status == TASK_COMPLETED:
            return None
        task.status = TASK_PENDING
        task.assigned_worker = ""
        task.goal = list(task.target_position)
        task.detail = str(detail)
        return task

    def mark_offline_workers(self, now: float, timeout_sec: float) -> List[RescueTaskRecord]:
        released: List[RescueTaskRecord] = []
        for worker in self.workers.values():
            if float(now) - worker.last_update > float(timeout_sec):
                worker.online = False
                task = self.release_worker(worker.name, "worker state timed out; task released")
                if task is not None:
                    released.append(task)
        return released

    def cancel_task(self, task_id: int, detail: str) -> Optional[RescueTaskRecord]:
        """Fail one task and immediately release its assigned worker."""

        task = self.tasks.get(int(task_id))
        if task is None or task.status == TASK_COMPLETED:
            return None
        worker = self.workers.get(task.assigned_worker)
        if worker is not None and worker.assigned_task == task.task_id:
            worker.assigned_task = None
        task.assigned_worker = ""
        task.status = TASK_FAILED
        task.detail = str(detail)
        return task

    def retry_task(self, task_id: int, detail: str) -> Optional[RescueTaskRecord]:
        """Return a non-completed task to the pending allocation queue."""

        task = self.tasks.get(int(task_id))
        if task is None or task.status == TASK_COMPLETED:
            return None
        worker = self.workers.get(task.assigned_worker)
        if worker is not None and worker.assigned_task == task.task_id:
            worker.assigned_task = None
        task.assigned_worker = ""
        task.goal = list(task.target_position)
        task.status = TASK_PENDING
        task.detail = str(detail)
        return task

    def remove_target_task(self, target_id: int) -> Optional[RescueTaskRecord]:
        """Remove the task linked to an operator-rejected target."""

        task_id = self._target_to_task.pop(int(target_id), None)
        if task_id is None:
            return None
        task = self.cancel_task(task_id, "target rejected by operator")
        self.tasks.pop(task_id, None)
        return task

    def clear_tasks(self, reset_ids: bool = True) -> None:
        """Clear mission tasks while retaining the configured worker registry."""

        for worker in self.workers.values():
            worker.assigned_task = None
        self.tasks.clear()
        self._target_to_task.clear()
        if reset_ids:
            self._next_task_id = 1
