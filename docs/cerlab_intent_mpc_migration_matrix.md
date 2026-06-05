# CERLAB and Intent-MPC Migration Matrix

Source roots:

- `external/dev/motion-planning/src/CERLAB-UAV-Autonomy`
- `external/dev/motion-planning/src/Intent-MPC`

This matrix compares the original upstream sources, not the earlier second-hand
copies. Counts ignore `.git`, build/install folders, and Python bytecode.

## Package Overlap

| Package | Classification | Same files | Changed files | CERLAB-only files | Intent-only files | Decision |
|---|---:|---:|---:|---:|---:|---|
| `global_planner` | identical | 38 | 0 | 0 | 0 | Already migrated as `xgc2_cerlab_global_planner`; do not duplicate for Intent. |
| `time_optimizer` | identical | 27 | 0 | 0 | 0 | Migrate once as `xgc2_cerlab_time_optimizer`. |
| `map_manager` | near-same | 26 | 2 | 0 | 0 | Keep one package; current static subset is `xgc2_cerlab_map_manager`, full migration needs dynamic map variant. |
| `tracking_controller` | near-same | 8 | 3 | 0 | 0 | Keep `xgc2_cerlab_tracking_controller`; evaluate Intent controller deltas before merging. |
| `remote_control` | view-only deltas | 6 | 0 | 10 | 2 | Package as runtime RViz/launch assets, not algorithm core. |
| `onboard_detector` | same base, service/config deltas | 23 | 12 | 1 | 2 | Migrate after isolating `vision_msgs` and detector model assets. |
| `trajectory_planner` | Intent-heavy extension | 75 | 6 | 0 | 766 | Split CERLAB polynomial/B-spline planner from Intent MPC/ACADO extension. |
| `autonomous_flight` | scenario wrapper split | 2 | 2 | 58 | 15 | Treat as launch/demo orchestration; migrate after planner/controller stack is complete. |
| `uav_simulator` | simulator assets | 139 | 10 | 2 | 37 | Move to simulator product, not planner. |
| `dynamic_predictor` | Intent-only | 0 | 0 | 0 | 15 | Migrate as `xgc2_intent_mpc_dynamic_predictor`. |

## Already Migrated

These packages are already in `products/ros1/planner/planner/cerlab`:

- `xgc2_cerlab_map_manager`
- `xgc2_cerlab_global_planner`
- `xgc2_cerlab_tracking_controller`

The first migration intentionally removed `dynamicMap`, `DEP`, and
`onboard_detector` coupling so that the first planner package could build and
publish without pulling perception/model assets into planner.

## Exact Duplicates

These should not be copied twice:

- `global_planner`: identical in both projects.
- `time_optimizer`: identical in both projects.

Recommended package names:

- `xgc2_cerlab_global_planner`
- `xgc2_cerlab_time_optimizer`

Intent-MPC should depend on these shared packages instead of creating
`xgc2_intent_mpc_global_planner` or `xgc2_intent_mpc_time_optimizer`.

## Near-Same Packages

### `map_manager`

Changed files:

- `include/map_manager/occupancyMap.cpp`
- `include/map_manager/occupancyMap.h`

Decision:

- Keep one `xgc2_cerlab_map_manager` package.
- Add full dynamic-map support in a second migration pass if needed.
- Avoid creating an Intent-specific map manager unless the two changed files
  prove to be behaviorally incompatible.

### `tracking_controller`

Changed files:

- `cfg/controller_param.yaml`
- `include/tracking_controller/trackingController.cpp`
- `include/tracking_controller/trackingController.h`

Decision:

- Keep one `xgc2_cerlab_tracking_controller` package.
- Inspect the Intent controller delta before merging. If it is only topic,
  gain, or reference-trajectory handling, fold it behind config/launch.

### `onboard_detector`

Intent adds:

- `srv/GetDynamicObstacles.srv`
- corrected `cfg/fake_detector_param.yaml` spelling

Changed files include detector params, fake/dynamic detector code, YOLO helper
scripts, and package metadata.

Decision:

- Migrate as `xgc2_cerlab_onboard_detector` or a more neutral
  `xgc2_cerlab_dynamic_detector`.
- Keep detector model weights and example images out of planner debs unless
  they are required runtime assets.
- This package is needed by full dynamic map and Intent predictor flows.

## Intent-Specific Extension

### `dynamic_predictor`

Intent-only package with:

- `dynamicPredictor.cpp/.h`
- `dynamic_predictor_node.cpp`
- `dynamic_predictor_fake_node.cpp`
- predictor launch/cfg/rviz

Depends on:

- `onboard_detector`
- `map_manager`

Decision:

- Migrate after `onboard_detector` is packaged.
- Use name `xgc2_intent_mpc_dynamic_predictor`.

### `trajectory_planner`

CERLAB and Intent share the basic planner package, but Intent adds a large MPC
extension:

- `mpcPlanner.cpp/.h`
- dynamic obstacle clustering
- ACADO generated solver code
- bundled qpOASES source
- bundled ACADO source tree
- `mpc_interactive.launch`

Decision:

- First migrate the CERLAB trajectory planner as
  `xgc2_cerlab_trajectory_planner`.
- Then decide whether Intent MPC is one package
  `xgc2_intent_mpc_trajectory_planner` or a split package pair:
  `xgc2_intent_mpc_dynamic_predictor` plus
  `xgc2_intent_mpc_trajectory_planner`.
- Do not mix bundled ACADO/qpOASES into the CERLAB package.

## Simulator-Only Assets

`uav_simulator` should move to a simulator product, not planner.

Target location:

- `products/ros1/simulator/gazebo-sim`

Suggested package:

- `xgc2_cerlab_uav_simulator` for shared simulator base
- optional `xgc2_intent_mpc_uav_simulator` only for Intent-only Livox, tunnel,
  generated world, and extra message assets

Intent-only simulator additions include:

- Livox lidar plugin and scan pattern CSVs
- `CustomPoint.msg`
- `LivoxCustomMsg.msg`
- generated-world scripts and configs
- tunnel and Mill19 worlds/models
- GPU/lidar URDF variants

## Migration Order

Recommended next batches:

1. `xgc2_cerlab_trajectory_planner`
   - CERLAB base trajectory planner only.
   - Depends on current `xgc2_cerlab_global_planner` and
     `xgc2_cerlab_map_manager`.

2. `xgc2_cerlab_time_optimizer`
   - Exact duplicate between both upstreams.
   - Depends on global planner, trajectory planner, and map manager.

3. `xgc2_cerlab_onboard_detector`
   - Needed for dynamic map and Intent predictor.
   - Requires deciding whether `vision_msgs` is packaged as a ROS apt
     dependency or replaced/isolated.

4. Full `xgc2_cerlab_map_manager` dynamic-map extension
   - Reintroduce dynamic map only after detector is available.

5. `xgc2_intent_mpc_dynamic_predictor`
   - Intent-only package.
   - Depends on detector and full/dynamic map support.

6. Intent MPC trajectory extension
   - ACADO/qpOASES handling must be isolated from CERLAB base planner.

7. Simulator packages under `products/ros1/simulator/gazebo-sim`
   - Move `uav_simulator` worlds, models, URDF, plugins, and Livox additions.

## Current Risk Items

- `vision_msgs` is required by `onboard_detector` upstream; verify availability
  in the CI image or package it separately.
- Intent trajectory planner bundles ACADO and qpOASES source; this needs a
  packaging decision before importing.
- `autonomous_flight` is mostly orchestration and should wait until planner,
  controller, detector, predictor, and simulator packages exist.
- Runtime assets should be preserved, but source/docs/test payloads should not
  be published in debs.

## Algorithm Integrity Notes

This migration should be driven by complete algorithm chains, not only package
or filename similarity.

### Shared Controller Correctness

Intent changes one line in `tracking_controller` that fixes velocity-error
derivative feedback:

- CERLAB computes `deltaVelError` from `velocityError - prevPosError`.
- Intent computes `deltaVelError` from `velocityError - prevVelError`.

This is a real control-law correction and should be carried into the shared
`xgc2_cerlab_tracking_controller`. Intent's extra simulation topic switch is a
separate runtime integration concern and should be parameterized only when the
simulator launch stack is migrated.

### Static Polynomial Planner Chain

The base CERLAB static chain is:

- `xgc2_cerlab_map_manager`
- `xgc2_cerlab_global_planner`
- `xgc2_cerlab_trajectory_planner`
- `xgc2_cerlab_time_optimizer`
- `xgc2_cerlab_tracking_controller`

`trajectory_planner` and `time_optimizer` should be migrated as a connected
unit. `time_optimizer` includes `trajectory_planner/bspline.h` and its
trajectory divider uses both `global_planner/KDTree.h` and
`map_manager/occupancyMap.h`, so importing it before the trajectory planner
would create a package that builds only by accident.

### Intent MPC Chain

Intent MPC is not only an added executable. The chain is:

- polynomial/B-spline reference path from `trajectory_planner`
- static obstacle clustering from `map_manager::occMap`
- dynamic obstacle history from `onboard_detector`
- intent prediction from `dynamic_predictor`
- MPC candidate generation and evaluation in `mpcPlanner`
- ACADO-generated solver plus qpOASES sources

The MPC extension should therefore wait until detector, dynamic map, and
predictor interfaces are present. Importing `mpcPlanner` first would either
stub out core inputs or silently change the planner's objective.

### Small Deltas With Runtime Meaning

Several small diffs are algorithmically relevant:

- `map_manager/occupancyMap.cpp`: Intent treats `prebuilt_map_directory` as
  relative to `autonomous_flight`; CERLAB treats it as already absolute. Product
  packages should use package-relative launch/config paths explicitly instead
  of inheriting either hardcoded developer path.
- `trajectory_planner/polyTrajOccMap.cpp`: Intent adds a fallback polynomial
  solve when corridor solving fails without PWL failsafe. This affects whether
  the downstream MPC receives a reference trajectory.
- `onboard_detector`: Intent adds out-of-range dynamic obstacle estimation,
  dynamic-obstacle history export, color-image YOLO association, and a
  `GetDynamicObstacles` service. These are part of the predictor/MPC data
  path, not cosmetic detector changes.

### Package Boundary Rule

Packages that are exact duplicates should be migrated once under the CERLAB
namespace. Packages that are behavior extensions should remain split until the
runtime graph is complete:

- shared: global planner, base trajectory planner, time optimizer, map manager,
  tracking controller
- detector/predictor extension: onboard detector, dynamic map, Intent predictor
- MPC extension: Intent trajectory planner/MPC solver and demo launch
- simulator extension: Gazebo worlds, URDF, plugins, Livox messages and models
