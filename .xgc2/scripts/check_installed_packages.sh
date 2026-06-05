#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-planner >/dev/null
dpkg -s ros-noetic-xgc2-planner-all >/dev/null
dpkg -s ros-noetic-xgc2-planner-common >/dev/null
dpkg -s ros-noetic-xgc2-gcopter >/dev/null
dpkg -s ros-noetic-xgc2-jps3d >/dev/null
dpkg -s ros-noetic-xgc2-cerlab-planner >/dev/null
dpkg -s ros-noetic-xgc2-intent-mpc >/dev/null
dpkg -s ros-noetic-xgc2-mader >/dev/null
dpkg -s ros-noetic-xgc2-robust-mader >/dev/null
dpkg -s ros-noetic-xgc2-ego-planner >/dev/null
dpkg -s ros-noetic-xgc2-fast-planner >/dev/null
dpkg -s ros-noetic-xgc2-mockamap >/dev/null
dpkg -s libboost-dev >/dev/null
dpkg -s libompl15 >/dev/null
dpkg -s libnlopt0 >/dev/null
dpkg -s libeigen3-dev >/dev/null
dpkg -s libyaml-cpp-dev >/dev/null

test "$(rospack find gcopter)" = "/opt/ros/${ROS_DISTRO}/share/gcopter"
test "$(rospack find jps3d)" = "/opt/ros/${ROS_DISTRO}/share/jps3d"
test "$(rospack find xgc2_cerlab_map_manager)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_map_manager"
test "$(rospack find xgc2_cerlab_global_planner)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_global_planner"
test "$(rospack find xgc2_cerlab_onboard_detector)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_onboard_detector"
test "$(rospack find xgc2_cerlab_tracking_controller)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_tracking_controller"
test "$(rospack find xgc2_cerlab_trajectory_planner)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_trajectory_planner"
test "$(rospack find xgc2_cerlab_time_optimizer)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_time_optimizer"
test "$(rospack find xgc2_cerlab_remote_control)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_remote_control"
test "$(rospack find xgc2_cerlab_autonomous_flight)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_autonomous_flight"
test "$(rospack find xgc2_intent_mpc_dynamic_predictor)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_intent_mpc_dynamic_predictor"
test "$(rospack find xgc2_intent_mpc_trajectory_planner)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_intent_mpc_trajectory_planner"
test "$(rospack find xgc2_intent_mpc_autonomous_flight)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_intent_mpc_autonomous_flight"
test "$(rospack find mader)" = "/opt/ros/${ROS_DISTRO}/share/mader"
test "$(rospack find rmader)" = "/opt/ros/${ROS_DISTRO}/share/rmader"
test "$(rospack find mader_msgs)" = "/opt/ros/${ROS_DISTRO}/share/mader_msgs"
test "$(rospack find rmader_msgs)" = "/opt/ros/${ROS_DISTRO}/share/rmader_msgs"
test "$(rospack find snapstack_msgs)" = "/opt/ros/${ROS_DISTRO}/share/snapstack_msgs"
test "$(rospack find quadrotor_msgs)" = "/opt/ros/${ROS_DISTRO}/share/quadrotor_msgs"
test "$(rospack find ego_planner)" = "/opt/ros/${ROS_DISTRO}/share/ego_planner"
test "$(rospack find fast_plan_manage)" = "/opt/ros/${ROS_DISTRO}/share/fast_plan_manage"
test "$(rospack find mockamap)" = "/opt/ros/${ROS_DISTRO}/share/mockamap"
test -x "/opt/ros/${ROS_DISTRO}/lib/gcopter/global_planning"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_global_planner/rrt_interactive_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_global_planner/rrt_star_interactive_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_map_manager/occupancy_map_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_map_manager/dynamic_map_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_onboard_detector/detector_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_onboard_detector/fake_detector_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_tracking_controller/tracking_controller_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_trajectory_planner/bspline_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_autonomous_flight/navigation_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_dynamic_predictor/xgc2_intent_mpc_dynamic_predictor_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_dynamic_predictor/xgc2_intent_mpc_dynamic_predictor_fake_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_trajectory_planner/mpc_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_autonomous_flight/mpc_navigation_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/mader/mader_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/rmader/rmader_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/ego_planner/ego_planner_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/fast_plan_manage/fast_planner_node"
test -f "/opt/ros/${ROS_DISTRO}/include/gcopter/gcopter/gcopter.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/gcopter/misc/visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/share/gcopter/launch/global_planning.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/jps_basis/data_type.h"
test -f "/opt/ros/${ROS_DISTRO}/include/jps_collision/map_util.h"
test -f "/opt/ros/${ROS_DISTRO}/include/jps_planner/jps_planner/jps_planner.h"
test -f "/opt/ros/${ROS_DISTRO}/include/jps_planner/distance_map_planner/distance_map_planner.h"
test -f "/opt/ros/${ROS_DISTRO}/lib/libjps_lib.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libdmp_lib.so"
test -f "/opt/ros/${ROS_DISTRO}/share/jps3d/cmake/jps3dConfig.cmake"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_global_planner/rrtOctomap.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_map_manager/occupancyMap.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_onboard_detector/dynamicDetector.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_tracking_controller/trackingController.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_trajectory_planner/bspline.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_cerlab_time_optimizer/timeOptimizer.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_intent_mpc_dynamic_predictor/dynamicPredictor.h"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_intent_mpc_trajectory_planner/mpcPlanner.h"
test -f "/opt/ros/${ROS_DISTRO}/share/xgc2_cerlab_global_planner/launch/rrt_interactive.launch"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(
  {
    find "/opt/ros/${ROS_DISTRO}/lib/gcopter" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_global_planner" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_map_manager" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_onboard_detector" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_tracking_controller" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_trajectory_planner" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_time_optimizer" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_cerlab_autonomous_flight" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_dynamic_predictor" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_trajectory_planner" -type f 2>/dev/null
    find "/opt/ros/${ROS_DISTRO}/lib/xgc2_intent_mpc_autonomous_flight" -type f 2>/dev/null
    printf '%s\n' \
      "/opt/ros/${ROS_DISTRO}/lib/libjps_lib.so" \
      "/opt/ros/${ROS_DISTRO}/lib/libdmp_lib.so"
  } | sort -u
)

echo "Installed package check passed"
