import threading
import unittest
from sar_yolo_detector.publication_lifecycle import PublicationGate

class LifecycleMetricsTest(unittest.TestCase):
    def test_independent_publishers_do_not_serialize_each_other(self):
        gate=PublicationGate();first=threading.Event();second=threading.Event();release=threading.Event()
        class Publisher:
            def __init__(self,event):self.event=event
            def publish(self,value):self.event.set();release.wait(2.)
        a,b=Publisher(first),Publisher(second)
        threads=[threading.Thread(target=lambda:gate.call(a.publish,None)),
                 threading.Thread(target=lambda:gate.call(b.publish,None))]
        try:
            for worker in threads:worker.start()
            self.assertTrue(first.wait(1.))
            self.assertTrue(second.wait(1.))
        finally:
            release.set()
            for worker in threads:worker.join(2.)
            gate.close()

    def test_shutdown_waits_for_inflight_publish_and_blocks_late_work(self):
        gate=PublicationGate()
        entered=threading.Event();release=threading.Event();closing=threading.Event();closed=threading.Event()
        messages=[]
        def publish(value):
            entered.set();release.wait(2.);messages.append(value)
        worker=threading.Thread(target=lambda:gate.call(publish,'inflight'))
        def finish():
            closing.set();gate.close();closed.set()
        closer=threading.Thread(target=finish)
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            closer.start();self.assertTrue(closing.wait(1.))
            self.assertFalse(closed.is_set())
        finally:
            release.set();worker.join(2.)
            if closer.ident is not None:closer.join(2.)
        self.assertTrue(closed.is_set())
        self.assertFalse(gate.call(messages.append,'late'))
        self.assertEqual(['inflight'],messages)

    def test_genuine_publish_error_is_not_hidden_as_shutdown(self):
        stopping=[False];gate=PublicationGate(lambda:stopping[0])
        def broken(value):raise RuntimeError('publish() to a closed topic')
        with self.assertRaises(RuntimeError):gate.call(broken,None)
        stopping[0]=True
        self.assertFalse(gate.call(broken,None))


    def test_stop_request_never_waits_but_outer_close_still_drains_publish(self):
        gate=PublicationGate();entered=threading.Event();release=threading.Event()
        requested=threading.Event();closed=threading.Event();messages=[]
        def publish(value):
            entered.set();release.wait(2.);messages.append(value)
        worker=threading.Thread(target=lambda:gate.call(publish,'inflight'))
        def signal_hook():
            gate.request_stop();requested.set()
        def cleanup():
            gate.close();closed.set()
        hook=threading.Thread(target=signal_hook);closer=threading.Thread(target=cleanup)
        worker.start()
        try:
            self.assertTrue(entered.wait(1.));hook.start()
            self.assertTrue(requested.wait(1.))
            self.assertFalse(gate.call(messages.append,'late'))
            closer.start();self.assertFalse(closed.is_set())
        finally:
            release.set();worker.join(2.);hook.join(2.)
            if closer.ident is not None:closer.join(2.)
        self.assertTrue(closed.is_set());self.assertEqual(['inflight'],messages)
