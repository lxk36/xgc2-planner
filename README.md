# xgc2-planner

ROS1 Noetic planner package repository for XGC2.

## Packages

- `gcopter`: GCOPTER global multicopter trajectory optimizer and demo launch.
- `jps3d`: JPS3D jump point search and distance-map planner libraries.
- `mader`: MADER multi-agent trajectory planner.
- `rmader`: Robust MADER multi-agent trajectory planner.
- `ego_planner`: EGO-Planner local trajectory planner.
- `fast_plan_manage`: Fast-Planner local trajectory planner.
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
