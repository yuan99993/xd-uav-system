"""Bounded evaluation logging; disk stalls must not block the UE capture tick."""
import queue
import threading


class TruthWriter:
    def __init__(self,path,maximum_pending=128,open_file=None):
        self.queue=queue.Queue(maximum_pending)
        self.error=None
        self.stream=(open_file or (lambda p:p.open('w',buffering=65536)))(path)
        self.worker=threading.Thread(target=self._run,daemon=True)
        self.worker.start()

    def write(self,line):
        if self.error is not None:raise RuntimeError('Evaluation log writer failed') from self.error
        try:self.queue.put_nowait(line)
        except queue.Full:raise RuntimeError('Evaluation log backlog exceeds capacity; trial invalid')

    def _run(self):
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                self.stream.write(item)
                self.stream.flush()
        except Exception as error:self.error=error
        finally:self.stream.close()

    def close(self):
        try:self.queue.put(None,timeout=2.)
        except queue.Full:raise RuntimeError('Evaluation log did not drain')
        self.worker.join(5.)
        if self.worker.is_alive():raise RuntimeError('Evaluation log writer stalled')
        if self.error is not None:raise RuntimeError('Evaluation log writer failed') from self.error
