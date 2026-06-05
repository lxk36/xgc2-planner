# xgc2-planner

ROS1 Noetic planner package repository for XGC2.

## Packages

- `gcopter`: GCOPTER global multicopter trajectory optimizer and demo launch.
- `jps3d`: JPS3D jump point search and distance-map planner libraries.
- `mader`: MADER multi-agent trajectory planner.
- `rmader`: Robust MADER multi-agent trajectory planner.
- `ego_planner`: EGO-Planner local trajectory planner.
- `fast_plan_manage`: Fast-Planner local trajectory planner.
- `mpc_planner*`: imported `tud-amr/mpc_planner` source, currently parked with `CATKIN_IGNORE` until solver generation and dependencies are packaged.
- `ros-noetic-xgc2-planner-common`: shared support packages used by planner stacks.
- `ros-noetic-xgc2-planner-all`: metapackage for all planner packages.
- `ros-noetic-xgc2-planner`: default metapackage that depends on `planner-all`.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-planner
```

The planner package depends on the map package `ros-noetic-xgc2-mockamap`, so
the procedural point-cloud map generator is installed automatically.

## Smoke Test

These commands check package discovery:

```bash
source /opt/ros/noetic/setup.bash
rospack find gcopter
rospack find jps3d
rospack find mader
rospack find rmader
rospack find ego_planner
rospack find fast_plan_manage
```

To run the demo:

```bash
source /opt/ros/noetic/setup.bash
roslaunch gcopter global_planning.launch
```

## Imported Candidates

`tud-amr/mpc_planner` has been imported as source packages only. It is not in
the default catkin build or APT package set yet because the upstream solver
generation output is not present in-tree:

- `mpc_planner_solver/solver.cmake`
- `mpc_planner_solver/mpc_planner_solver-extras.cmake`
- generated Acados/FORCES solver source and headers
- `mpc_planner_modules/modules.cmake`

The intended XGC2 path is Acados-only generation before catkin build. FORCES Pro
should not be required for public CI or APT publication.

`tkkim-robot/safe_control` was evaluated as a safety-filter candidate, but the
upstream repository currently has no license file. Its source is therefore not
redistributed in this public product until explicit redistribution permission is
available.
