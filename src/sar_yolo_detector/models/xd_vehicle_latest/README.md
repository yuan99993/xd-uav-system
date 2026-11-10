# XD vehicle SmartTracker model

`xd_vehicle_latest.pt` is copied from `src/train/weights/best.pt` (training run
`train`). The filename `latest` is a historical alias, not the active runtime
model. It is the default of the archived
`archive/launch/xd_smart_tracker_integration.launch`.

The current `xd_uav_track/launch/vision_tracking_stack.launch` defaults to train7.
Launch arguments can override that default. Read the shared detector's latched
`~model_status` topic for the loaded artifact path, SHA-256, backend and device.

- Classes: `0=car`, `1=ar-car`, `2=tank`
- SHA-256: `932c04d47e426143e9fa5084a4ca6a6ff49c1a3aff70fa2b44c643b165e5151f`
- Training run: `train` (rollback target)
- Deployment scope: XD ground-vehicle SmartTracker integration only. Aircraft,
  thermal, maritime, wildfire, and segmentation profiles retain their
  task-specific models.

The train7 artifact used by the current production launch default is stored as
`../xd_vehicle_train7/xd_vehicle_train7.pt` with SHA-256
`8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54`.
