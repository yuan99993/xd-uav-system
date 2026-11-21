#!/usr/bin/python3
"""Run the civilian RGB regression ladder, halting at the first failed FOV."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--python-executable', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        parser.error('Use a new or empty output directory')
    suite = Path(__file__).resolve().with_name('run_ue_stress_suite.py')
    cases = ['civil_car_fast', 'civil_drone_fast', 'ground_2mps_h60', 'occlusion_6mps_h60']
    stages = []
    initial_hashes = None
    for angle in (90, 75, 60, 45):
        config = output / f'fov_{angle}.json'
        config.write_text(json.dumps({'schedule': [[0, angle]], 'actuator': {'initial_deg': angle}}))
        folder = output / f'fov_{angle}'
        with (output / f'fov_{angle}.log').open('w') as log:
            code = subprocess.call([sys.executable, str(suite), '--project', str(args.project),
                                    '--python-executable', args.python_executable, '--output', str(folder),
                                    '--launch-file', 'ue_civil_moving_targets.launch', '--cases', *cases,
                                    '--pose-source', 'px4', '--seconds', '30', '--no-record',
                                    '--roi-padding', '2', '--fov-profile', str(config), '--require-stage-pass'],
                                   stdout=log, stderr=subprocess.STDOUT)
        manifest = folder / 'suite_manifest.json'
        hashes = json.loads(manifest.read_text())['source_sha256'] if manifest.is_file() else None
        if initial_hashes is None:
            initial_hashes = hashes
        results_file = folder / 'suite_results.json'
        results = json.loads(results_file.read_text()) if results_file.is_file() else []
        passed = code == 0 and hashes is not None and hashes == initial_hashes and (
            [v['case_id'] for v in results] == cases and all(v.get('fov_stage_acceptance') for v in results))
        stages.append(dict(fov_deg=angle, passed=passed, process_exit_code=code,
                           tested_cases=[v['case_id'] for v in results],
                           failing_cases=[v['case_id'] for v in results if not v.get('fov_stage_acceptance')]))
        (output / 'ladder_results.json').write_text(json.dumps(stages, indent=2) + '\n')
        if not passed:
            print(f'Stopped at {angle}°: {output / "ladder_results.json"}')
            return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
