from pathlib import Path
import sys
import tempfile
import threading
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ue'))
from benchmark_truth_writer import TruthWriter


class TruthWriterTest(unittest.TestCase):
    def test_close_preserves_every_record_in_order(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'truth.jsonl';writer=TruthWriter(path)
            rows=['{"seq":%d}\n'%i for i in range(100)]
            for row in rows:writer.write(row)
            writer.close()
            self.assertEqual(path.read_text(),''.join(rows))

    def test_disk_stall_never_blocks_capture_and_backlog_invalidates_trial(self):
        entered=threading.Event();release=threading.Event();written=[]
        class SlowDisk:
            def write(self,line):entered.set();release.wait(2);written.append(line)
            def flush(self):pass
            def close(self):pass
        writer=TruthWriter(None,maximum_pending=1,open_file=lambda _:SlowDisk())
        try:
            writer.write('first');self.assertTrue(entered.wait(1))
            writer.write('second')
            with self.assertRaisesRegex(RuntimeError,'trial invalid'):writer.write('overflow')
        finally:release.set();writer.close()
        self.assertEqual(written,['first','second'])

    def test_disk_error_is_reported_and_never_silently_drops_truth(self):
        class FailedDisk:
            def write(self,_):raise OSError('disk unavailable')
            def close(self):pass
        writer=TruthWriter(None,open_file=lambda _:FailedDisk());writer.write('record')
        writer.worker.join(1)
        with self.assertRaisesRegex(RuntimeError,'writer failed'):writer.write('next')
        with self.assertRaisesRegex(RuntimeError,'writer failed'):writer.close()


if __name__=='__main__':unittest.main()
