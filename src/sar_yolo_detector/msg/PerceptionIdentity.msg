# Immutable provenance attached to every planning-facing perception event.
# session_uuid changes on every process start unless explicitly injected by the
# mission launcher.  The tuple (mission_id, uav_id, session_uuid) names one
# producer incarnation and prevents local track IDs colliding across aircraft.
string mission_id
string uav_id
string session_uuid
string sensor_id
string profile
string model_version
string model_sha256
string calibration_version

# Coordinate convention used by localized positions, for example
# LOCAL_ENU:map or ECEF:EPSG4978.  Bearing-only observations use CAMERA_RAY.
string coordinate_contract
