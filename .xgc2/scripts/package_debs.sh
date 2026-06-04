#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
PACKAGE_GROUP="gcopter"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.0.0-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --package-group)
      PACKAGE_GROUP="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "--output-dir is required" >&2
  exit 1
fi

if [[ "${PACKAGE_GROUP}" != "meta" && -z "${INSTALL_ROOT}" ]]; then
  echo "--install-root is required for package group ${PACKAGE_GROUP}" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${description}
EOF
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_pkg}" "${dst_root}"
}

build_ros_group_deb() {
  local package="$1"
  local depends="$2"
  local description="$3"
  shift 3
  local ros_packages=("$@")

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  for ros_pkg in "${ros_packages[@]}"; do
    copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  done

  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_meta_deb() {
  local package="$1"
  local depends="$2"
  local description="$3"
  local pkg_root="${BUILD_DIR}/${package}"

  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

common_pkg="ros-noetic-xgc2-planner-common"
gcopter_pkg="ros-noetic-xgc2-gcopter"
mader_pkg="ros-noetic-xgc2-mader"
rmader_pkg="ros-noetic-xgc2-robust-mader"
ego_pkg="ros-noetic-xgc2-ego-planner"
fast_pkg="ros-noetic-xgc2-fast-planner"
all_meta_pkg="ros-noetic-xgc2-planner-all"
meta_pkg="ros-noetic-xgc2-planner"

case "${PACKAGE_GROUP}" in
  common)
    build_ros_group_deb \
      "${common_pkg}" \
      "libeigen3-dev, libglpk40, libgmp10, libmpfr6, libqt5x11extras5, ros-noetic-actionlib-msgs, ros-noetic-geometry-msgs, ros-noetic-interactive-markers, ros-noetic-message-runtime, ros-noetic-nav-msgs, ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-rqt-gui, ros-noetic-rqt-gui-py, ros-noetic-rviz, ros-noetic-sensor-msgs, ros-noetic-shape-msgs, ros-noetic-std-msgs, ros-noetic-std-srvs, ros-noetic-tf2-eigen, ros-noetic-tf2-geometry-msgs, ros-noetic-tf2-ros, ros-noetic-trajectory-msgs, ros-noetic-visualization-msgs" \
      "XGC2 shared dependencies for planner packages" \
      behavior_selector cgal_wrapper decomp_ros_msgs decomp_ros_utils decomp_util eigen_stl_containers graph_msgs quadrotor_msgs rviz_visual_tools separator snapstack_msgs view_controller_msgs
    ;;
  gcopter)
    build_ros_group_deb \
      "${gcopter_pkg}" \
      "libeigen3-dev, libompl15, ros-noetic-roscpp, ros-noetic-std-msgs, ros-noetic-geometry-msgs, ros-noetic-sensor-msgs, ros-noetic-visualization-msgs, ros-noetic-rviz, ros-noetic-rqt-plot, ros-noetic-xgc2-mockamap" \
      "XGC2 GCOPTER trajectory optimizer for ROS1" \
      gcopter
    ;;
  mader)
    build_ros_group_deb \
      "${mader_pkg}" \
      "${common_pkg} (= ${VERSION}), libnlopt0, ros-noetic-gazebo-msgs, ros-noetic-jsk-rviz-plugins, ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-sensor-msgs" \
      "XGC2 MADER multi-agent trajectory planner for ROS1" \
      mader mader_msgs
    ;;
  robust-mader)
    build_ros_group_deb \
      "${rmader_pkg}" \
      "${common_pkg} (= ${VERSION}), libcgal-dev, libnlopt0, ros-noetic-jsk-rviz-plugins, ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-sensor-msgs" \
      "XGC2 Robust MADER multi-agent trajectory planner for ROS1" \
      rmader rmader_msgs
    ;;
  ego-planner)
    build_ros_group_deb \
      "${ego_pkg}" \
      "${common_pkg} (= ${VERSION}), libeigen3-dev, libnlopt-cxx0, libnlopt0, ros-noetic-cmake-modules, ros-noetic-message-runtime, ros-noetic-nav-msgs, ros-noetic-pcl-ros, ros-noetic-roscpp, ros-noetic-roslib, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-tf" \
      "XGC2 EGO-Planner local trajectory planner for ROS1" \
      ego_bspline_opt ego_local_sensing ego_map_generator ego_path_searching ego_plan_env ego_planner ego_traj_utils ego_waypoint_generator
    ;;
  fast-planner)
    build_ros_group_deb \
      "${fast_pkg}" \
      "${common_pkg} (= ${VERSION}), libeigen3-dev, libnlopt-cxx0, libnlopt0, ros-noetic-cmake-modules, ros-noetic-message-runtime, ros-noetic-nav-msgs, ros-noetic-pcl-ros, ros-noetic-roscpp, ros-noetic-roslib, ros-noetic-rospy, ros-noetic-std-msgs, ros-noetic-tf" \
      "XGC2 Fast-Planner local trajectory planner for ROS1" \
      bspline fast_bspline_opt fast_local_sensing fast_map_generator fast_path_searching fast_plan_env fast_plan_manage fast_traj_utils fast_waypoint_generator poly_traj
    ;;
  meta)
    build_meta_deb \
      "${all_meta_pkg}" \
      "${common_pkg} (= ${VERSION}), ${gcopter_pkg} (= ${VERSION}), ${mader_pkg} (= ${VERSION}), ${rmader_pkg} (= ${VERSION}), ${ego_pkg} (= ${VERSION}), ${fast_pkg} (= ${VERSION})" \
      "XGC2 ROS1 complete planner package set"
    build_meta_deb \
      "${meta_pkg}" \
      "${all_meta_pkg} (= ${VERSION})" \
      "XGC2 ROS1 default planner metapackage"
    ;;
  *)
    echo "unknown package group: ${PACKAGE_GROUP}" >&2
    exit 1
    ;;
esac

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
