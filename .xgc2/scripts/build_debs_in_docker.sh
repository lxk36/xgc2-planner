#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
PLANNER_GROUP="${PLANNER_GROUP:-gcopter}"
DOCKER_RUN_ARGS="${DOCKER_RUN_ARGS:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --planner-group)
      PLANNER_GROUP="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
# shellcheck disable=SC2206
extra_docker_args=(${DOCKER_RUN_ARGS})
docker run --rm \
  "${extra_docker_args[@]}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e PLANNER_GROUP="${PLANNER_GROUP}" \
  -v "${REPO_ROOT}:/workspace/planner:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libcgal-dev \
      libboost-dev \
      libeigen3-dev \
      libglpk-dev \
      libgmp-dev \
      libmpfr-dev \
      libnlopt-cxx-dev \
      libnlopt-dev \
      libogre-1.9-dev \
      libompl-dev \
      libqt5x11extras5-dev \
      libyaml-cpp-dev \
      pkg-config \
      python3-yaml \
      rsync \
      ros-noetic-actionlib-msgs \
      ros-noetic-cmake-modules \
      ros-noetic-cv-bridge \
      ros-noetic-gazebo-msgs \
      ros-noetic-geometry-msgs \
      ros-noetic-image-transport \
      ros-noetic-interactive-markers \
      ros-noetic-jsk-rviz-plugins \
      ros-noetic-mavros \
      ros-noetic-mavros-msgs \
      ros-noetic-message-filters \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-nav-msgs \
      ros-noetic-octomap-ros \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-roscpp \
      ros-noetic-roslint \
      ros-noetic-roslib \
      ros-noetic-rospack \
      ros-noetic-rospy \
      ros-noetic-rqt-gui \
      ros-noetic-rqt-gui-py \
      ros-noetic-rqt-plot \
      ros-noetic-rviz \
      ros-noetic-sensor-msgs \
      ros-noetic-shape-msgs \
      ros-noetic-std-msgs \
      ros-noetic-std-srvs \
      ros-noetic-tf \
      ros-noetic-tf2-eigen \
      ros-noetic-tf2-geometry-msgs \
      ros-noetic-tf2-ros \
      ros-noetic-trajectory-msgs \
      ros-noetic-visualization-msgs

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src

    copy_common() {
      rsync -a /workspace/planner/common/ /workspace/work/src/
      rsync -a /workspace/planner/mader_common/ /workspace/work/src/
    }

    case "${PLANNER_GROUP}" in
      common)
        copy_common
        ;;
      gcopter)
        rsync -a --delete /workspace/planner/gcopter/ /workspace/work/src/gcopter/
        ;;
      jps3d)
        rsync -a --delete /workspace/planner/jps3d/ /workspace/work/src/jps3d/
        ;;
      cerlab-planner)
        rsync -a /workspace/planner/cerlab/ /workspace/work/src/
        ;;
      mader)
        copy_common
        rsync -a /workspace/planner/mader/ /workspace/work/src/
        ;;
      robust-mader)
        copy_common
        rsync -a /workspace/planner/rmader/ /workspace/work/src/
        ;;
      ego-planner)
        copy_common
        rsync -a /workspace/planner/ego_planner/ /workspace/work/src/
        ;;
      fast-planner)
        copy_common
        rsync -a /workspace/planner/fast_planner/ /workspace/work/src/
        ;;
      *)
        echo "unknown planner group: ${PLANNER_GROUP}" >&2
        exit 1
        ;;
    esac

    cd /workspace/work
    source /opt/ros/noetic/setup.bash

    catkin_make \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DUSE_GUROBI=OFF \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DUSE_GUROBI=OFF \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/planner/.xgc2/scripts/package_debs.sh \
      --package-group "${PLANNER_GROUP}" \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
