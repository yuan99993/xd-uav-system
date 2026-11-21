"""Shutdown-only publication guard; no dependency on tracker state locks."""
import threading

class PublicationGate:
    """Finish an in-flight publish before shutdown closes its publishers."""
    def __init__(self,shutdown_requested=lambda:False):
        self.lock=threading.RLock()
        self.publish_locks={}
        self.closed=False
        self.shutdown_requested=shutdown_requested

    def request_stop(self):
        """Nonblocking admission stop: never wait for an in-flight publisher."""
        self.closed=True

    def close(self):
        self.request_stop()
        with self.lock:locks=tuple(self.publish_locks.values())
        for lock in locks:
            with lock:pass

    def call(self,publish,message):
        if self.closed or self.shutdown_requested():return False
        key=id(getattr(publish,'__self__',publish))
        with self.lock:
            lock=self.publish_locks.get(key)
            if lock is None:
                lock=threading.RLock();self.publish_locks[key]=lock
        with lock:
            if self.closed or self.shutdown_requested():return False
            try:publish(message)
            except Exception:
                if self.closed or self.shutdown_requested():return False
                raise
            return True

