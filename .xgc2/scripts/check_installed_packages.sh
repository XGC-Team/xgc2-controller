#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-reference" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-utils" >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s libxgc2-geometry-dev >/dev/null
dpkg -s xgc2-acados >/dev/null
test "$(rospack find multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_controller"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_controller/config/uav_nmpc.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_controller/launch/uav_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/drone_controller.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/uav/state_machine/custom1_state.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/SetRuntimeParameters.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/multirotor_controller/multirotor_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_controller_uav_nmpc_runtime.so"
roslaunch --files multirotor_controller uav_nmpc_controller.launch >/tmp/xgc2-controller-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/multirotor_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_controller_uav_nmpc_runtime.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
