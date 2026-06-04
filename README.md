# xgc2-planner

ROS1 Noetic planner package repository for XGC2.

## Packages

- `gcopter`: GCOPTER global multicopter trajectory optimizer and demo launch.
- `mader`: MADER multi-agent trajectory planner.
- `rmader`: Robust MADER multi-agent trajectory planner.
- `ros-noetic-xgc2-mader-common`: shared MADER/RMADER support packages.
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
rospack find mader
rospack find rmader
```

To run the demo:

```bash
source /opt/ros/noetic/setup.bash
roslaunch gcopter global_planning.launch
```
