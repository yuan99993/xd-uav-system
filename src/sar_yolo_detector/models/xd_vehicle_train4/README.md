# train4 XD vehicle model archive

`xd_vehicle_train4.pt` is a selectable historical vehicle detector model.
The production shared-detector launch currently defaults to train7; this file
does not describe the model loaded by a running process.  Query
`sar_yolo_multi_source_detection/model_status` or its `list_models` service for
the actual model content identity.

- Source: `src/train4/weights/best.pt`
- Classes (verified from the actual weight metadata): `0=car`, `1=ar-car`, `2=tank`, `3=m142`
- Prebuilt FP16 engine: `xd_vehicle_train4.engine` (local TensorRT 10.1.0 host)
- SHA-256: `12e26bd00dfe18bd5f2e3cbc579aaad911f12b05239bb3ada3656deacd0d90e5`
