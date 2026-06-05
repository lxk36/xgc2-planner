#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

required_files=(
  ".xgc2/product.yml"
  ".xgc2/scripts/build_debs_in_docker.sh"
  ".xgc2/scripts/check_installed_packages.sh"
  ".xgc2/scripts/check_package_compliance.sh"
  ".xgc2/scripts/check_ros_packages.sh"
  ".xgc2/scripts/package_debs.sh"
  ".xgc2/scripts/publish_apt_repo.sh"
  ".github/workflows/build-debs.yml"
  "README.md"
  "gcopter/CMakeLists.txt"
  "gcopter/package.xml"
  "gcopter/launch/global_planning.launch"
  "gcopter/config/global_planning.yaml"
  "jps3d/CMakeLists.txt"
  "jps3d/package.xml"
  "jps3d/include/jps_planner/jps_planner/jps_planner.h"
  "common/quadrotor_msgs/package.xml"
  "fast_planner/fast_plan_manage/package.xml"
  "fast_planner/fast_plan_manage/CMakeLists.txt"
  "ego_planner/ego_planner/package.xml"
  "ego_planner/ego_planner/CMakeLists.txt"
  "mader/mader/CMakeLists.txt"
  "mader/mader/package.xml"
  "mader/mader_msgs/package.xml"
  "rmader/rmader/CMakeLists.txt"
  "rmader/rmader/package.xml"
  "rmader/rmader_msgs/package.xml"
  "mader_common/snapstack_msgs/package.xml"
  "mader_common/decomp_util/package.xml"
)

for file in "${required_files[@]}"; do
  test -f "${REPO_ROOT}/${file}" || {
    echo "missing required file: ${file}" >&2
    exit 1
  }
done

grep -q "id: xgc2-planner" "${REPO_ROOT}/.xgc2/product.yml"
grep -q "<name>gcopter</name>" "${REPO_ROOT}/gcopter/package.xml"
grep -q "<name>jps3d</name>" "${REPO_ROOT}/jps3d/package.xml"
grep -q "<name>fast_plan_manage</name>" "${REPO_ROOT}/fast_planner/fast_plan_manage/package.xml"
grep -q "<name>ego_planner</name>" "${REPO_ROOT}/ego_planner/ego_planner/package.xml"
grep -q "find_package(ompl REQUIRED)" "${REPO_ROOT}/gcopter/CMakeLists.txt"
grep -q "PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)" "${REPO_ROOT}/jps3d/CMakeLists.txt"
grep -q "libboost-dev" "${REPO_ROOT}/.xgc2/scripts/build_debs_in_docker.sh"
grep -q "libyaml-cpp-dev" "${REPO_ROOT}/.xgc2/scripts/build_debs_in_docker.sh"
grep -q "ros-noetic-xgc2-mockamap" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-planner-common" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-jps3d" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-robust-mader" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-ego-planner" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "ros-noetic-xgc2-fast-planner" "${REPO_ROOT}/.xgc2/scripts/package_debs.sh"
grep -q "branches:" "${REPO_ROOT}/.github/workflows/build-debs.yml"
grep -q "noetic" "${REPO_ROOT}/.github/workflows/build-debs.yml"

echo "Package compliance check passed"
