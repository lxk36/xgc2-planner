# xgc2_cerlab_map_manager

XGC2-packaged subset of Zhefan Xu's CERLAB `map_manager`.

This first migration keeps the occupancy map, ESDF map, and raycast service
used by the CERLAB RRT planner. The dynamic map wrapper is intentionally
excluded because it depends on `onboard_detector`, which belongs in a later
perception-oriented migration.

Launch examples:

```bash
roslaunch xgc2_cerlab_map_manager occupancy_map.launch
roslaunch xgc2_cerlab_map_manager esdf_map.launch
```
