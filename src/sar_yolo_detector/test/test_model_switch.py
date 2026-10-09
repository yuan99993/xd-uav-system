#!/usr/bin/env python3
"""Deterministic preparation failure and batch-boundary semantics without CUDA."""
from pathlib import Path
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

from sar_yolo_detector.model_registry import _sha256
from sar_yolo_detector.model_switch import ModelSwitch, SwitchError


class Backend:
    def __init__(self): self.closed = False
    def unload_model(self): self.closed = True


class ModelSwitchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        pt, engine = root / "m.pt", root / "m.engine"
        pt.write_bytes(b"pt"); engine.write_bytes(b"engine")
        self.item = dict(engine_prepared=True, engine_path=str(engine), pt_path=str(pt),
            pt_sha256=_sha256(pt), engine_sha256=_sha256(engine),
            engine_source_model_sha256=_sha256(pt), class_names=["tank"])
        self.wake = threading.Event(); self.done = threading.Event(); self.errors = []
        self.backend = Backend()
        self.memory = (2 * 1024**3, 4 * 1024**3)
        self.switch = ModelSwitch({"m":self.item}, lambda item:(self.backend, {}),
            lambda:self.memory, self.wake.set, self.failed)
        self.addCleanup(self.switch.close)
        self.command = SimpleNamespace(model_id="m", request_id="1")

    def failed(self, command, code, reason):
        self.errors.append(code); self.done.set()

    def test_prepared_model_is_not_committed_by_loader(self):
        self.switch.submit(self.command)
        self.assertTrue(self.wake.wait(2))
        with self.assertRaises(SwitchError) as error:self.switch.submit(self.command)
        self.assertEqual("BUSY", error.exception.code)
        self.assertIs(self.backend, self.switch.take_ready()[2][0])
        self.assertIsNotNone(self.switch.pending)
        self.switch.complete()
        self.assertIsNone(self.switch.pending)

    def test_missing_engine_never_builds_online(self):
        self.item["engine_prepared"] = False
        self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2)); self.assertEqual(["NOT_PREPARED"],self.errors)

    def test_hash_tampering_rejected(self):
        Path(self.item["engine_path"]).write_bytes(b"tampered")
        self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2)); self.assertEqual(["HASH_MISMATCH"],self.errors)

    def test_memory_precheck_preserves_old_model(self):
        self.memory = (64 * 1024**2, 4 * 1024**3)
        self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2)); self.assertEqual(["INSUFFICIENT_MEMORY"],self.errors)
        self.assertFalse(self.backend.closed)

    def test_failed_warmup_keeps_old_model(self):
        def bad(item): raise RuntimeError("warmup failed")
        self.switch.loader = bad
        self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2)); self.assertEqual(["LOAD_FAILED"],self.errors)
        self.assertIsNone(self.switch.pending)

    def test_post_load_memory_check_releases_candidate(self):
        def loaded(item):
            self.memory = (0, 4 * 1024**3)
            return self.backend, {}
        self.switch.loader = loaded
        self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2)); self.assertTrue(self.backend.closed)
        self.assertEqual(["INSUFFICIENT_MEMORY"],self.errors)

    def test_allocator_oom_is_classified_without_committing(self):
        def loaded(item):raise RuntimeError("CUDA out of memory")
        self.switch.loader=loaded;self.switch.submit(self.command)
        self.assertTrue(self.done.wait(2));self.assertEqual(["INSUFFICIENT_MEMORY"],self.errors)

    def test_shutdown_during_load_discards_candidate(self):
        entered = threading.Event(); release = threading.Event()
        def loaded(item):
            entered.set(); release.wait(1)
            return self.backend, {}
        self.switch.loader = loaded
        self.switch.submit(self.command); self.assertTrue(entered.wait(1))
        with self.switch.lock:self.switch.closed=True
        release.set(); self.assertTrue(self.done.wait(2)); self.assertTrue(self.backend.closed)


if __name__ == "__main__": unittest.main()
