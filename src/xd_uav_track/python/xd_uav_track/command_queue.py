"""One bounded FIFO for mode, model and selection transactions.

Callbacks enqueue immutable copies. The executor waits for asynchronous model
completion before touching the next command; optimistic versions are validated
at execution, never silently rebased to a newer state.
"""
from collections import deque
from copy import deepcopy
import hashlib
import threading


class CommandQueue:
    def __init__(self, execute, capacity=16):
        self.execute = execute
        self.capacity = max(1, min(64, int(capacity)))
        self.condition = threading.Condition()
        self.items = deque()
        self.requests = {}
        self.active_request_id = ""
        self.closed = False
        self.errors = 0
        self.worker = threading.Thread(target=self._run, name="perception_transactions", daemon=True)
        self.worker.start()

    def submit(self, kind, command):
        fingerprint = hashlib.sha256((kind + str(command)).encode("utf-8")).hexdigest()
        with self.condition:
            if self.closed: return "UNAVAILABLE"
            old = self.requests.get(command.request_id)
            if old is not None:
                return "DUPLICATE" if old == fingerprint else "REQUEST_CONFLICT"
            if len(self.requests) >= self.capacity: return "BUSY"
            self.requests[command.request_id] = fingerprint
            self.items.append((kind, deepcopy(command)))
            self.condition.notify()
        return "QUEUED"

    def snapshot(self):
        with self.condition:
            return self.active_request_id, len(self.items)

    def _run(self):
        while True:
            with self.condition:
                while not self.items and not self.closed:self.condition.wait()
                if self.closed:return
                kind, command = self.items.popleft()
                self.active_request_id = command.request_id
            try:
                self.execute(kind, command)
            except Exception:
                # An executor should report its own failure, but a malformed
                # request must not permanently kill the transaction worker.
                self.errors += 1
            finally:
                with self.condition:
                    self.requests.pop(command.request_id, None)
                    self.active_request_id = ""

    def close(self):
        with self.condition:
            self.closed = True
            self.items.clear()
            self.condition.notify_all()
        if self.worker is not threading.current_thread():self.worker.join(timeout=3)
