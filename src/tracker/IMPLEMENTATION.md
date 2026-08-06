# Tracker implementation policy

`tracker_node` (C++) is the production implementation. It owns the existing
selected-target outputs and additionally provides persistent multi-target
states, explicit target selection, calibrated line-of-sight computation and
standard ROS diagnostics.

`scripts/tracker_node.py` is retained only as a compatibility/reference
implementation for older `ExternalInput` workflows. Product launch files must
use `type="tracker_node"`, which resolves to the compiled C++ executable.

The compatibility interfaces remain:

- `~external_input`
- `~detection_candidates`
- `~tracking_output`
- `~normalized_error`

The additive C++ interfaces are:

- `~track_states` (`tracker/TrackStateArray`)
- `~select_track` (`tracker/SelectTrack`)
- `/diagnostics`
