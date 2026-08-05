# Estimator quarantine recovery change

## Why this exists

Stage 3 SEAD V5 evidence captured a repeatable estimator recovery deadlock at
touchdown. A correction that drifted outside the innovation gate was
quarantined, but after quarantine it had to pass the same gate against the
still-predicting old filter before it could correct that filter. Persistent,
valid measurements could therefore be quarantined forever.

Evidence: `/tmp/sead_v5_long_probe.bag` from 2026-08-05. The flight phase was
healthy; at touchdown all UAVs briefly isolated vertical corrections and uav1
remained invalid after disarm.

## Incremental code change

`src/multi_source_estimator_node.cpp` retains all normal innovation thresholds.
Only a correction already leaving quarantine enters the new path:

1. Validate timestamp, frame and covariance as before.
2. Require `recovery_min_samples` mutually consistent samples for at least
   `recovery_stable_time`.
3. Reacquire only that correction's position, velocity, yaw or yaw-rate state
   component; do not reinitialize unrelated filter components.
4. Return to the unchanged normal innovation gate.

A single spike cannot trigger reacquisition. The implementation is marked by
the `recoverySampleReady()`, `reacquireFilterComponent()` and
`finishRecovery()` helpers and an in-code safety comment.

## Verification and rollback

`test/quarantine_recovery.test` reproduces a persistent height step, requires
the estimator to become invalid and then recover near the new height, and
checks that one isolated spike does not cause invalidation/reacquisition.

Rollback is limited to removing that test registration and the three recovery
helpers/call site. No SEAD, controller, manager, PX4 or Gazebo behavior is
changed.
