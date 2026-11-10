# Tracklet stitching

`xd_uav_track` now has an optional identity-layer stitcher for the case where a
local camera tracker removes one trajectory and later creates a new local
track ID for the same object. The detector's ID is not rewritten and no ROS
message fields were added.

## Identity and state handling

- A camera-local ID remains owned by its `MultiTrackManager`.
- The node's global identity map supplies the stable ID used by selection and
  control. A successful stitch remaps the new local ID to the archived global
  ID; an existing locked ID is not replaced.
- Global identity records keep a bounded appearance gallery, latest metric
  world position and covariance, and a constant-velocity estimate derived
  from timestamped world observations.
- Records expire by wall-clock TTL and are capped by
  `tracker/tracklet_stitching/maximum_identities`.

## Acceptance gates

A candidate must have a confirmed local track, a valid metric world position,
compatible class, a positive observation-time gap within the archive TTL, a
bounded physical displacement/speed, and a diagonal Mahalanobis innovation
inside the configured gate. The position prediction uses the archived world
velocity and covariance growth. Same-camera matching additionally needs a
matching appearance prototype or consistent world velocity; cross-camera
matching requires a matching appearance prototype as well as metric geometry.
Raw pixels are never treated as a shared coordinate system.

Eligible candidates are ranked. A score below the absolute threshold or an
insufficient best-versus-second-best margin rejects the stitch as ambiguous.
The same identity must remain the best candidate across consecutive confirmed
observations before the local-to-global mapping changes. During this interval
the candidate is labelled `stitch_pending` and is not control-ready. Ambiguity
is labelled `stitch_ambiguous`; it is not forced into the old identity.

## Configuration

Defaults are in `config/track.yaml` under `tracker/tracklet_stitching`. The
Gazebo image-only `tracking_benchmark.yaml` overlay disables the feature because
its pixel ground truth is not a calibrated world-frame localization source.
The matcher intentionally declines to stitch when metric position is missing;
same-camera ego-motion-compensated image fallback is not enabled until such a
compensated observation contract is available.

The unit suite `test_tracklet_stitcher` covers inertial prediction, ambiguous
near-ties, missing world/appearance evidence, active same-source exclusion,
archive expiry, class/confirmation gates, and consecutive-frame confirmation.
These are algorithm tests, not a real-data identity-accuracy certification.
