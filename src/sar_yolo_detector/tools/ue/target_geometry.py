"""Image-stage geometry contract. Classification alone never establishes depth.

World rays are directions in ue_render_ned, not target positions. The same
pixel servo handles air, ground and unclassified manual selections. A future
metric producer must provide and validate its own measurement provenance.
"""
import math
import numpy as np

GROUND_CLASSES = frozenset({'car', 'tank', 'truck', 'bus', 'van', 'vehicle'})
AIR_CLASSES = frozenset({'drone', 'uav', 'aircraft', 'airplane', 'helicopter'})


def target_domain(class_name):
    name = str(class_name or '').strip().lower()
    if name in GROUND_CLASSES:
        return 'ground_candidate'
    if name in AIR_CLASSES:
        return 'airborne'
    return 'unknown'


def sightline_evidence(state, ray, camera_position=None):
    ray = np.asarray(ray, dtype=float)
    norm = np.linalg.norm(ray)
    if ray.shape != (3,) or not np.isfinite(ray).all() or norm < 1e-8:
        raise ValueError('Invalid measured camera sight line')
    domain = target_domain(state.get('class_name'))
    origin = None
    if camera_position is not None:
        position = np.asarray(camera_position, dtype=float)
        if position.shape == (3,) and np.isfinite(position).all():
            origin = position.tolist()
    return dict(schema_version=1, measurement_stamp=float(state['stamp']),
        frame_id=state['frame_id'], world_frame='ue_render_ned',
        track_id=int(state['track_id']), class_id=int(state.get('class_id', -1)),
        class_name=str(state.get('class_name', '')), target_domain=domain,
        observation_kind='image_sightline', modality='rgb',
        ray_world_unit=(ray / norm).tolist(), camera_origin_ned_m=origin,
        range_valid=False, target_position_valid=False, ground_projection_applied=False,
        metric_policy=('validated_ground_contact_and_surface_required' if domain=='ground_candidate'
            else 'independent_range_or_triangulation_required' if domain=='airborne'
            else 'domain_and_metric_evidence_required'))
