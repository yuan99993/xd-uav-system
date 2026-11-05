"""Deployment integrity: actual trained tensors and approved native code roots."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
BUNDLE = PACKAGE/'models/dfine_s_v3_16e'


class ModelArtifactTest(unittest.TestCase):
    def test_actual_v3_weights_and_configuration_are_integrity_pinned(self):
        manifest=json.loads((BUNDLE/'manifest.json').read_text())
        self.assertEqual(manifest['class_names'],['car','tank','drone'])
        self.assertEqual(manifest['files']['model.safetensors'],
            'd4b408dd28f6e9d570304dce1bf342f8d1590193f05adca03f16dbc6ad1650bd')
        for name,expected in manifest['files'].items():
            self.assertEqual(hashlib.sha256((BUNDLE/name).read_bytes()).hexdigest(),expected,name)
        for name,expected in manifest['source_repository']['files'].items():
            self.assertEqual(hashlib.sha256((PACKAGE/'third_party/D-FINE'/name).read_bytes()).hexdigest(),expected,name)


@unittest.skipIf(importlib.util.find_spec('torch') is None, 'Requires the D-FINE runtime')
class NativeLoaderTest(unittest.TestCase):
    def test_package_uri_resolves_to_this_package(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import native_repository
        self.assertEqual(native_repository('dfine','package://sar_yolo_detector/third_party/D-FINE'),PACKAGE/'third_party/D-FINE')

    def test_arbitrary_manifest_python_root_is_rejected(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import native_repository
        with self.assertRaisesRegex(ValueError,'Unapproved'):
            native_repository('dfine','/tmp/other-native-code')

    def test_corrupt_weights_fail_before_model_construction(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import load_bundle
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            (folder/'manifest.json').write_bytes((BUNDLE/'manifest.json').read_bytes())
            (folder/'config.json').write_bytes((BUNDLE/'config.json').read_bytes())
            (folder/'model.safetensors').write_bytes(b'corrupt artifact')
            with self.assertRaisesRegex(ValueError,'artifact integrity mismatch'):
                load_bundle(folder/'manifest.json','cpu')


if __name__ == '__main__':
    unittest.main()
