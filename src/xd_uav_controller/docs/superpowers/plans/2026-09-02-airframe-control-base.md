# Airframe Control Base Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the two `_new` ROS packages from binary vehicle selection to airframe-aware, VTOL-capable control while retaining existing multirotor and fixed-wing behavior.

**Architecture:** `xd_uav_control_manager` owns configured airframe, PX4-observed flight regime, requested regime, lifecycle action, and transition supervision. `xd_uav_controller` consumes that state and selects exactly one multirotor or fixed-wing backend; transition regimes continue the source backend until PX4 confirms the target state.

**Tech Stack:** ROS1 Noetic, catkin, roscpp, MAVROS, gtest/rostest, C++17.

**Spec:** `/home/kzy/xd_uav_system_control_base_refactor_for_codex.md`

## Global Constraints

- Modify only `src/xd_uav_controller` and `src/xd_uav_control_manager`.
- Keep old packages read-only.
- Rename the ROS packages to `xd_uav_controller` and `xd_uav_control_manager`.
- Preserve legacy `vehicle_type` parameters and message fields as deprecated compatibility inputs.
- Keep one controller command publisher and one active backend.
- Flight regime is observed from PX4 `ExtendedState`; service acceptance never completes a transition.
- Write a failing test before each new behavior and run it before production changes.

---

### Task 1: Baseline and ROS package rename

**Files:**
- Modify: `xd_uav_controller/package.xml`, `CMakeLists.txt`, `launch/*.launch`, tests and source includes
- Modify: `xd_uav_control_manager/package.xml`, `CMakeLists.txt`, `launch/*.launch`, tests and source includes

**Produces:** Two uniquely named catkin packages that can coexist with the read-only legacy packages.

- [ ] Write a package-discovery test command that lists both old and `_new` packages.
- [ ] Run it before changes and record that `_new` directories currently collide on package name.
- [ ] Rename both package manifests and CMake projects; change all generated message namespaces, dependencies, `find` calls and launch `pkg` attributes to `_new` names.
- [ ] Build the two renamed packages in an isolated catkin workspace and run the existing tests.

### Task 2: Shared airframe and regime types

**Files:**
- Create: `xd_uav_controller/include/xd_uav_controller/control_types.h`
- Create: `xd_uav_controller/test/test_control_types.cpp`
- Modify: `xd_uav_controller/CMakeLists.txt`

**Produces:** `AirframeType`, `FlightRegime`, `RequestedRegime`, `VehicleAction`, `BackendId`, parsers, compatibility parsing and validation helpers.

- [ ] Add a failing gtest for `multirotor`, `fixedwing`, `fixed_wing`, `vtol`, `tiltrotor`, invalid values and legal airframe/regime combinations.
- [ ] Run the test and verify compilation fails because the shared API does not exist.
- [ ] Implement the minimal header-only type API and validation table.
- [ ] Re-run the test and confirm pass.

### Task 3: State, command and transition wire interfaces

**Files:**
- Modify: `xd_uav_controller/msg/ControlState.msg`, `ControlCommand.msg`
- Create: `xd_uav_control_manager/srv/SetFlightRegime.srv`
- Modify: both `CMakeLists.txt`, package dependencies, source includes and tests

**Produces:** Wire-level airframe/regime/action/generation fields and a manager-only VTOL transition service.

- [ ] Add failing tests that assert the generated constants and fields are present and that legacy fields still compile.
- [ ] Build to verify missing generated interfaces fail.
- [ ] Add message constants/fields and service generation.
- [ ] Build and run interface tests.

### Task 4: ControlManager airframe, action and VTOL transition supervision

**Files:**
- Create: `xd_uav_control_manager/include/xd_uav_control_manager/vehicle_adapter.h`
- Create: `xd_uav_control_manager/include/xd_uav_control_manager/vtol_vehicle_adapter.h`
- Create: `xd_uav_control_manager/src/vtol_vehicle_adapter.cpp`
- Modify: `xd_uav_control_manager/src/control_manager_node.cpp`, config, launch and tests

**Produces:** Airframe parsing, observed regime mapping, requested-regime state, `SetFlightRegime`, MAVROS transition requests, timeout and diagnostic state.

- [ ] Add failing unit/rostests for MC/FW/transition mapping, rejected service, accepted-but-unconfirmed request and transition timeout.
- [ ] Run tests and verify they fail on absent service/state behavior.
- [ ] Implement the adapter and connect it to the existing `ExtendedState` subscriber, client and diagnostics.
- [ ] Run tests and confirm state changes only after `ExtendedState` confirmation.

### Task 5: Controller backend resolver and compatibility handover

**Files:**
- Create: `xd_uav_controller/include/xd_uav_controller/controller_backend.h`
- Create: `xd_uav_controller/include/xd_uav_controller/backend_resolver.h`
- Create: `xd_uav_controller/src/backend_resolver.cpp`
- Modify: `xd_uav_controller/src/controller_node.cpp`, config and tests

**Produces:** A single resolver selecting multirotor/fixed-wing backend from observed regime, source-backend continuity during transition and generation-safe command echoing.

- [ ] Add failing tests for HOVER, FORWARD_FLIGHT, both transition states, UNKNOWN and stale regime generation rejection.
- [ ] Run tests and verify resolver symbols/behavior are missing.
- [ ] Implement resolver and route existing control math through it without changing math functions.
- [ ] Run multirotor/fixed-wing regression tests plus resolver tests.

### Task 6: Lifecycle action protocol and VTOL landing sequence

**Files:**
- Modify: `xd_uav_controller/srv/InternalCommand.srv`, `src/controller_node.cpp`
- Modify: `xd_uav_control_manager/src/control_manager_node.cpp`, tests

**Produces:** Manager-owned action generation/status, FW-to-MC-before-vertical-landing sequencing and compatibility fields.

- [ ] Add failing tests for VTOL land from FW waiting for MC before issuing vertical landing and for stale action generations.
- [ ] Run tests and verify existing manager immediately starts the legacy landing command.
- [ ] Implement action generation/status propagation and the landing sub-phase.
- [ ] Run lifecycle tests and existing landing tests.

### Task 7: VTOL configuration, launch and end-to-end verification

**Files:**
- Create: `xd_uav_controller/config/vtol.yaml`, `xd_uav_control_manager/config/vtol.yaml`
- Create: `xd_uav_control_manager/launch/vtol_system.launch`
- Modify: existing controller/manager launches and tests

**Produces:** VTOL launch that loads common, multirotor, fixed-wing and VTOL-specific configuration with namespaced MAVROS transition service.

- [ ] Add failing launch/config tests proving `fixed_wing` maps to `fixedwing.yaml` and VTOL loads both backend configurations.
- [ ] Run tests and verify the VTOL launch/configuration is absent.
- [ ] Implement configuration and launch wiring.
- [ ] Build, run all unit/rostests, then run the available SITL smoke test if the local PX4 setup is available.
