# Aircraft COCO baseline

`yolo11n.pt` is the Ultralytics YOLO11n COCO checkpoint used by the primary
scout SmartTracker profile. COCO class `4` is `airplane`; the task-candidate
profile filters every other class before publishing to the planner boundary.

SHA-256:

```text
0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1  yolo11n.pt
```

This is a generic COCO baseline, not an acceptance-tested small-aircraft,
counter-UAS, thermal-aircraft, or long-range aerial model. Keep
`model_validated:=false` until representative camera, altitude, range,
weather, target-size, false-positive and latency tests pass. In particular,
the class name alone is not evidence that the model can reliably recognize
small drones or aircraft viewed from another aircraft.

The scout launch therefore defaults to `bearing_only`. A monocular bounding box
does not contain enough information to determine an airborne target's range.
Produce a world-coordinate task only after a calibrated range sensor,
multi-aircraft bearing intersection, multi-view estimator, or another validated
3-D source supplies the missing range. `terrain` ray intersection is appropriate
only when the detected target is known to lie on the terrain surface.

Ultralytics states that YOLO11 models are offered under AGPL-3.0 and Enterprise
licenses. See `THIRD_PARTY_NOTICES.md` and `LICENSES/Ultralytics-AGPL-3.0.txt`;
deployment owners are responsible for selecting and complying with the
applicable license.
