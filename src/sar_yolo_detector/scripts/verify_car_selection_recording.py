#!/usr/bin/python3
"""Validate and export the original selected RGB capture for civilian replay."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import rosbag
from cv_bridge import CvBridge
from sar_yolo_detector.rgb_recording import load_manifest, calibration_index, validate_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = load_manifest(args.bag)
    selection = json.loads((args.evaluation/'selection.json').read_text())
    header = selection['capture_header']
    selected_ns = header['stamp']['secs']*10**9+header['stamp']['nsecs']
    args.output.mkdir(parents=True, exist_ok=False)
    found = None
    delivered = False
    configuration = []
    with rosbag.Bag(str(args.bag)) as bag:
        infos, transforms = calibration_index(bag, manifest['source'])
        for topic, msg, _ in bag.read_messages():
            if topic == manifest['source']['image_topic'] and msg.header.stamp.to_nsec() == selected_ns:
                validate_frame(msg, infos[selected_ns], transforms[selected_ns], manifest['source'])
                if msg.header.seq != header['seq']:
                    raise ValueError('Selected image sequence differs')
                rgb = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
                if not cv2.imwrite(str(args.output/'selected_raw.png'), rgb):
                    raise OSError('Failed to export selected pixels')
                found = dict(capture_nsec=selected_ns, seq=msg.header.seq,
                             raw_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),
                             K=list(infos[selected_ns].K), width=msg.width, height=msg.height,
                             roi=selection['initial_box'])
            if topic.endswith('/fusion_evidence'):
                value = json.loads(msg.data)
                h = value.get('capture_header', {})
                s = h.get('stamp', {})
                if s.get('secs', 0)*10**9+s.get('nsecs', 0) == selected_ns:
                    delivered = True
            if topic.endswith('/state') and hasattr(msg, 'model_path'):
                entry = dict(model_path=msg.model_path, backend=msg.backend, device=msg.device)
                if entry not in configuration:
                    configuration.append(entry)
    for entry in configuration:
        path=Path(entry['model_path'])
        if path.is_file():
            digest=hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(4*1024*1024),b''):
                    digest.update(chunk)
            entry['model_sha256']=digest.hexdigest()
    passed = bool(found and delivered and manifest['summary']['first_capture_nsec'] < selected_ns)
    report = dict(passed=passed, bag_sha256=manifest['bag_sha256'], selected_capture=found,
                  initial_frame_processed_evidence=delivered, model_runtime=configuration,
                  first_capture_nsec=manifest['summary']['first_capture_nsec'])
    (args.output/'selection_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    if not passed:
        raise RuntimeError('Initial selected frame recording/processing validation failed')


if __name__ == '__main__':
    main()
