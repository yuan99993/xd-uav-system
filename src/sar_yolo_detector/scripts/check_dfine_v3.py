#!/usr/bin/env python3
"""Verify the packaged model/native code and the four-mode Python runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    parser.add_argument('--load-model', action='store_true')
    parser.add_argument('--image', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    if not (package/'package.xml').exists():
        import rospkg
        package = Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    sys.path.insert(0, str(package/'python'))
    path = package/'models/dfine_s_v3_16e/manifest.json'
    manifest = json.loads(path.read_text())
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    import yaml
    config = yaml.safe_load((package/'config/dfine_v3.yaml').read_text())['SmartTracker']
    assert sha(path) == config['SMART_TRACKER_MODEL_SHA256'], 'Manifest pin mismatch'
    assert manifest['class_names'] == ['car', 'tank', 'drone']
    for name, expected in manifest['files'].items():
        assert sha(path.parent/name) == expected, 'Model integrity mismatch: '+name
    for name, expected in manifest['source_repository']['files'].items():
        assert sha(package/'third_party/D-FINE'/name) == expected, 'Native code mismatch: '+name
    result = dict(model='D-FINE-S V3', model_path=str(path), manifest_sha256=sha(path),
                  weights_sha256=manifest['files']['model.safetensors'], artifact_integrity=True)
    if args.load_model or args.image:
        import cv2
        import torch
        assert hasattr(cv2, 'HOGDescriptor'), 'OpenCV HOG missing'
        for name in ('TrackerKCF_create', 'TrackerCSRT_create'):
            assert hasattr(cv2, name) or hasattr(getattr(cv2, 'legacy', None), name), 'OpenCV contrib '+name+' missing'
        if args.device == 'cuda':
            assert torch.cuda.is_available(), 'CUDA unavailable; provision CUDA or explicitly test with --device cpu'
        from sar_yolo_detector.pixeagle.backends.dfine_backend import DFineBackend
        from sar_yolo_detector.pixeagle.backends.detection_backend import DevicePreference
        config.update(SMART_TRACKER_MODELS_ROOT=str(package), SMART_TRACKER_FALLBACK_TO_CPU=False,
                      SMART_TRACKER_USE_GPU=args.device == 'cuda',
                      SMART_TRACKER_TRACKER_CONFIG_PATH=str(package/'config/vehicle_botsort.yaml'))
        torch.set_num_threads(2)
        backend = DFineBackend(config)
        result['runtime'] = backend.load_model(str(path), DevicePreference(args.device), fallback_enabled=False)
        if args.image:
            image = cv2.imread(str(args.image))
            assert image is not None, 'Unreadable test image'
            for i in range(3):
                _, rows = backend.detect_and_track(image, conf=.1, iou=.6, max_det=30,
                    tracker_args={'_capture_stamp':float(i+1)})
            assert any(row.track_id_is_stable for row in rows), 'No stable BoT-SORT track in smoke image'
            result['detections'] = [dict(id=row.track_id, stable=row.track_id_is_stable,
                class_id=row.class_id, score=row.confidence, box=list(row.aabb_xyxy)) for row in rows]
            result['stages'] = backend.stage_trace()
        backend.unload_model()
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
