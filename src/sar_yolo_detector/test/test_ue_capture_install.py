"""The UE installer must preserve existing startup code and be relocatable."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class UECaptureInstallTest(unittest.TestCase):
    def test_native_plugin_manifest_and_install(self):
        import json
        import importlib.util
        script=Path(__file__).resolve().parents[1]/'scripts/install_ue_capture.py'
        spec=importlib.util.spec_from_file_location('capture_installer',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        plugin=script.parents[1]/'ue_plugin/SarCameraStream'
        manifest=module.validate_stream_plugin(plugin)
        with tempfile.TemporaryDirectory() as folder:
            project=Path(folder);(project/'test.uproject').write_text('{}')
            subprocess.check_call([sys.executable,str(script),'--project',str(project),
                '--install-stream-plugin'],stdout=subprocess.DEVNULL)
            self.assertTrue((project/'Plugins/SarCameraStream/Binaries/Win64/UnrealEditor-SarCameraStream.dll').is_file())
            self.assertEqual(json.loads((project/'test.uproject').read_text())['Plugins'],
                [dict(Name='SarCameraStream',Enabled=True)])
            modules=project/'Engine/Binaries/Win64/UnrealEditor.modules'
            modules.parent.mkdir(parents=True);modules.write_text(json.dumps(dict(BuildId='different')))
            with self.assertRaisesRegex(ValueError,'BuildId mismatch'):
                module.validate_stream_plugin(plugin,project)
            modules.write_text(json.dumps(dict(BuildId=manifest['build_id'])))
            module.validate_stream_plugin(plugin,project)

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
            self.assertTrue((project/'Scripts/AirGround/virtual_gimbal.py').is_file())
            self.assertTrue((project/'Scripts/AirGround/scene.json').is_file())


    def test_roslaunch_remappings_are_not_capture_arguments(self):
        script=Path(__file__).resolve().parents[1]/'tools/ue/ue_native_bridge.py'
        output=subprocess.check_output([sys.executable,str(script),'--ros','--help',
            '__name:=bridge_argument_check','__log:=/tmp/bridge_argument_check.log'],stderr=subprocess.STDOUT)
        self.assertIn(b'--virtual-gimbal',output)



if __name__=='__main__':unittest.main()
