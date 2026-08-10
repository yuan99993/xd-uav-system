# Follower production and safety policy

`follower_node` is the production C++ implementation. The Python node remains
only as a compatibility/reference implementation and must not be selected by a
product launch file.

## Control boundary

The follower produces aircraft-follow intent; it is not a gimbal actuator
controller. `control_output_backend` selects exactly one output path:

- `command_only`: publish `FollowerCommand` for an XD/product adapter. This is
  the product default and does not write an aircraft actuator endpoint.
- `mavros_body`: publish `mavros_msgs/PositionTarget` in `FRAME_BODY_NED` after
  lease and MAVROS connected/armed/mode gates pass.
- `mrs_velocity`: publish one MRS velocity reference after lease and controller
  readiness gates pass.
- `mavros_twist_legacy`: compatibility path for old deployments only.

The deprecated MAVROS/MRS boolean parameters are inferred only when
`control_output_backend` is absent. Enabling both is a fail-closed
configuration error.

## Authorization sequence

Product configuration requires a bounded lease:

1. Acquire or renew `~manage_control_lease` with the configured backend.
2. Call `~start` with `data: true`.
3. Renew the lease before it expires.
4. Call `~stop`, release the lease, or use `~emergency_stop` when done.

Lease expiry disables following and zeros the public command intent. Emergency
stop is sticky: start/stop/profile changes cannot release it.

## Input and output invariants

- Product mode requires a valid capture timestamp and rejects out-of-order or
  future tracker samples.
- Predicted/unconfirmed/high-uncertainty samples remain fail-closed.
- Direct MAVROS output requires a fresh connected, armed and correctly moded
  flight state by default.
- Every command exposes `control_state`, `invalid_reason`, backend,
  authorization and platform readiness. The same state is available through
  `FollowerStatus` and `/diagnostics`.
