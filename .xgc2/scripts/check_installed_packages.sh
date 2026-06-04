#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-planner >/dev/null
dpkg -s ros-noetic-xgc2-gcopter >/dev/null
dpkg -s ros-noetic-xgc2-mockamap >/dev/null
dpkg -s libompl15 >/dev/null
dpkg -s libeigen3-dev >/dev/null

test "$(rospack find gcopter)" = "/opt/ros/${ROS_DISTRO}/share/gcopter"
test "$(rospack find mockamap)" = "/opt/ros/${ROS_DISTRO}/share/mockamap"
test -x "/opt/ros/${ROS_DISTRO}/lib/gcopter/global_planning"
test -f "/opt/ros/${ROS_DISTRO}/include/gcopter/gcopter/gcopter.hpp"
test -f "/opt/ros/${ROS_DISTRO}/include/gcopter/misc/visualizer.hpp"
test -f "/opt/ros/${ROS_DISTRO}/share/gcopter/launch/global_planning.launch"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/gcopter" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
