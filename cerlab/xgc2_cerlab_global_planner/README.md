# xgc2_cerlab_global_planner

XGC2-packaged subset of Zhefan Xu's CERLAB `global_planner`.

This package keeps the common RRT/RRT* planner code shared by
`CERLAB-UAV-Autonomy` and `Intent-MPC`, renamed into the XGC2 ROS namespace.
The dynamic exploration planner (`DEP`) is intentionally excluded from this
first migration because it pulls in dynamic mapping and detector dependencies.

Launch examples:

```bash
roslaunch xgc2_cerlab_global_planner rrt_interactive.launch
roslaunch xgc2_cerlab_global_planner rrt_star_interactive.launch
```
