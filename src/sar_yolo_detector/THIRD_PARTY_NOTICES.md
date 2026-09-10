# Third-party notices

## PixEagle-derived SmartTracker components

Files under `python/sar_yolo_detector/pixeagle/` are derived from PixEagle by
Alireza Ghaderi and contributors. PixEagle is distributed under the Apache
License 2.0. The original project is <https://github.com/alireza787b/PixEagle>.

The transplant retains the detection normalization, geometry, ROI validation,
motion prediction, Kalman tracking, appearance matching, robust tracking-state
manager, coordinate/gimbal helpers, command intent, yaw-rate smoothing, backend
contract, and SmartTracker behavior. Imports, configuration injection, model
integrity handling, and ROS integration were adapted for `sar_yolo_detector`.

The complete Apache License 2.0 text is distributed with this package as
`LICENSES/PixEagle-Apache-2.0.txt`.

## Ultralytics runtime and YOLO11 model

The optional SmartTracker runtime depends on the separately installed
`ultralytics` Python package. The aircraft COCO baseline at
`models/aircraft_coco/yolo11n.pt` is an Ultralytics YOLO11 model. Ultralytics
states that its software and models are available under AGPL-3.0 and Enterprise
licenses. The AGPL-3.0 text supplied by the installed Ultralytics 8.4.118
distribution is included as `LICENSES/Ultralytics-AGPL-3.0.txt`.

Deployments using an Enterprise license must retain their own applicable
license records. This notice does not claim that the generic COCO model is
validated for small-aircraft or airborne-to-airborne recognition.

## Torchreid and OSNet person ReID checkpoint

The optional deep appearance backend uses `torchreid==0.2.5` and its OSNet
implementation to extract person re-identification embeddings.  The bundled
`models/person_reid/osnet_x0_25_market1501.pt` artifact is the Torchreid
Market-1501 model-zoo checkpoint.  Torchreid/OSNet are MIT-licensed; the
upstream project and license are available at
<https://github.com/KaiyangZhou/deep-person-reid>.  This package verifies the
checkpoint SHA-256 at startup and does not download model files implicitly.
