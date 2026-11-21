import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

PACKAGE=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('civil_entry',str(PACKAGE/'scripts/run_civil_target_test.py'))
ENTRY=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(ENTRY)


class CivilEntryTests(unittest.TestCase):
    def test_profiles_distinguish_class_from_motion_and_have_only_civilian_targets(self):
        for name,category,motion in [('drone_static','drone','stationary'),('car_static','car','stationary'),
                ('car_fast','car','moving'),('drone_fast','drone','moving')]:
            p=ENTRY.scenario_profile(PACKAGE,name)
            self.assertEqual((p['target_class'],p['motion_state']),(category,motion))
            scene=json.loads(Path(p['scene']).read_text())
            primary=next(e for e in scene['entities'] if e['id']==scene['benchmark']['primary_entity'])
            if motion=='stationary':self.assertNotIn('trajectory',primary)
            else:self.assertEqual(primary['trajectory']['kind'],'sine')

    def test_all_model_categories_entry_requires_operator_selection(self):
        profile=ENTRY.scenario_profile(PACKAGE,'model_targets')
        self.assertEqual(profile['target_class'],'model_supported')
        self.assertEqual(profile['lock_class_policy'],'all_model_categories_operator_selected')
        for name in ['car_static','car_fast','drone_fast','drone_static']:
            self.assertEqual(ENTRY.scenario_profile(PACKAGE,name)['lock_class_policy'],'all_model_categories_operator_selected')

    def test_unknown_profile_is_rejected_and_px4_command_preserves_paired_gimbal(self):
        with self.assertRaises(ValueError):ENTRY.scenario_profile(PACKAGE,'invalid')
        p=ENTRY.scenario_profile(PACKAGE,'car_fast')
        args=SimpleNamespace(benchmark=False,project=Path('/tmp/UE project'),python_executable='/tmp/model env/bin/python',
            pose_source='px4',seconds=360,output=None)
        cmd=ENTRY.command(PACKAGE,p,args)
        self.assertIn('pose_source:=px4',cmd)
        self.assertIn('ue_project:=/tmp/UE project',cmd)
        self.assertIn('gimbal_config:='+p['gimbal_config'],cmd)

    def test_benchmark_cannot_start_without_output_and_tracks_scenario_identity(self):
        p=ENTRY.scenario_profile(PACKAGE,'drone_fast')
        args=SimpleNamespace(benchmark=True,project=Path('/tmp/pro1'),python_executable='/tmp/python',
            pose_source='px4',seconds=60,output=None,roi_padding=2,record=False)
        with self.assertRaises(ValueError):ENTRY.command(PACKAGE,p,args)
        args.output=Path('/tmp/new-results');cmd=ENTRY.command(PACKAGE,p,args)
        self.assertEqual(cmd[cmd.index('--cases')+1],p['case_id'])
        self.assertIn('--no-record',cmd)
