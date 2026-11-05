"""The UE installer must preserve existing startup code and be relocatable."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class UECaptureInstallTest(unittest.TestCase):
    def test_preserves_existing_startup_and_is_idempotent(self):
        script=Path(__file__).resolve().parents[1]/'scripts/install_ue_capture.py'
        with tempfile.TemporaryDirectory() as folder:
            project=Path(folder)/'Project With Spaces';project.mkdir()
            (project/'sample.uproject').write_text('{}')
            init=project/'Content/Python/init_unreal.py';init.parent.mkdir(parents=True)
            original='print("existing user startup")\n';init.write_text(original)
            for _ in range(2):
                subprocess.check_call([sys.executable,str(script),'--project',str(project),'--install-startup'],stdout=subprocess.DEVNULL)
            content=init.read_text()
            self.assertTrue(content.startswith(original))
            self.assertEqual(content.count('# BEGIN SAR UE CAPTURE'),1)
            self.assertNotIn(str(project),content)
            self.assertTrue(init.with_suffix('.py.sar-backup').exists())
            self.assertTrue((project/'Scripts/AirGround/frame_protocol.py').is_file())
            self.assertTrue((project/'Scripts/AirGround/scene.json').is_file())


if __name__=='__main__':unittest.main()
