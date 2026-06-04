# gcopter

ROS1 Noetic port of the GCOPTER global multicopter trajectory optimizer.

The package builds the `global_planning` node and can start the local
`mockamap` simulator package to publish an occupied-point cloud map on
`/voxel_map`. RViz goals are read from `/move_base_simple/goal` by default.

The build requires system Eigen3 and OMPL development files. On Ubuntu 20.04
Noetic, they are provided by `libeigen3-dev` and `libompl-dev`.

Build GCOPTER and its demo map generator from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="mockamap;gcopter"
```

Run without RViz:

```bash
source devel/setup.bash
roslaunch gcopter global_planning.launch start_rviz:=false
```
