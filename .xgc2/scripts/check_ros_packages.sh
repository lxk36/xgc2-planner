#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${ROS_PACKAGE_CHECK_WORK_DIR:-${REPO_ROOT}/.work/package-tests}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

rm -rf "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel"
mkdir -p "${WORK_DIR}/src"
rsync -a --delete "${REPO_ROOT}/gcopter/" "${WORK_DIR}/src/gcopter/"
rsync -a --delete "${REPO_ROOT}/jps3d/" "${WORK_DIR}/src/jps3d/"

cd "${WORK_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCATKIN_ENABLE_TESTING=ON

catkin_make install \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCATKIN_ENABLE_TESTING=OFF

source "${WORK_DIR}/devel/setup.bash"
test "$(rospack find gcopter)" = "${WORK_DIR}/src/gcopter"
test "$(rospack find jps3d)" = "${WORK_DIR}/src/jps3d"
test -f "${WORK_DIR}/install/include/jps_basis/data_type.h"
test -f "${WORK_DIR}/install/include/jps_collision/map_util.h"
test -f "${WORK_DIR}/install/include/jps_planner/jps_planner/jps_planner.h"
test -f "${WORK_DIR}/install/include/jps_planner/distance_map_planner/distance_map_planner.h"
test -f "${WORK_DIR}/install/lib/libjps_lib.so"
test -f "${WORK_DIR}/install/lib/libdmp_lib.so"
test -f "${WORK_DIR}/install/share/jps3d/cmake/jps3dConfig.cmake"

echo "ROS package check passed"
