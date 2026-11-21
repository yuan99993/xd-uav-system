"""Fixed-lens evaluation using capture and applied-command logs."""
import json
import math

import numpy as np


def evaluate_stage(result, capture, angle, require_occlusion):
    """Record measurable prerequisites and fail closed on missing data."""
    def read(name):
        path = capture / name
        return json.loads(path.read_text()) if path.is_file() else []

    start, end = result['scoring_window_wall']
    frames = [row for row in read('frame_timing.json') if start <= row['capture_wall'] <= end]
    correct = {round(stamp * 1e6) for stamp in result['correct_capture_stamps']}
    sequences = {row['sequence'] for row in read('control_timing.json')
                 if round(row['capture_stamp'] * 1e6) in correct}
    response = [row['capture_to_actuator_ms'] for row in read('actuator_timing.json')
                if row['sequence'] in sequences and math.isfinite(row['capture_to_actuator_ms'])
                and row['capture_to_actuator_ms'] >= 0.]
    p95 = float(np.percentile(response, 95)) if response else None
    largest_error = max((abs(row['fov'] - angle) for row in frames), default=None)
    occurrences = result['occlusion_recovery']
    identity = result['initial_selected_id']
    checks = {
        'initial_lock': result['motion_started_with_lock'],
        'valid_truth': result['evaluation_valid'],
        'measured_lens': largest_error is not None and largest_error <= .05,
        'correct_lock_fraction': result['correct_state_fraction'] >= .9,
        'identity': result['identity_numbers'] == [identity] and result['id_switches'] == 0,
        'box_alignment': result['centre_error_p95_norm'] is not None and result['centre_error_p95_norm'] <= .10,
        'centering': result['true_target_centre_offset_p95_norm'] is not None
                     and result['true_target_centre_offset_p95_norm'] <= .25,
        'visibility': require_occlusion or result['visible_capture_fraction'] >= .95,
        'bounded_gaps': result['visible_maximum_gap_sec'] is not None and result['visible_maximum_gap_sec'] <= .5,
        'no_hidden_lock': result['hidden_current_states'] == 0,
        'false_lock': result['false_lock_fraction_of_current'] <= .01,
        'actuator_latency': len(response) >= 5 and p95 <= 600.,
        'occlusion_recovered': not require_occlusion or bool(occurrences) and all(
            v['same_id_recovery_sec'] is not None and v['same_id_recovery_sec'] <= .5 for v in occurrences),
    }
    result.update(requested_fixed_fov_deg=angle, measured_fov_max_error_deg=largest_error,
                  measured_fov_samples=len(frames), capture_to_applied_p95_ms=p95,
                  capture_to_applied_samples=len(response), fov_stage_checks=checks,
                  fov_stage_acceptance=all(checks.values()))
