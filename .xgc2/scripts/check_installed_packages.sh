#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-hover-thrust" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-utils" >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s xgc2-acados >/dev/null
xgc2_acados_version="$(dpkg-query -W -f='${Version}' xgc2-acados)"
dpkg --compare-versions "${xgc2_acados_version}" ge "0.1.0-3~focal"
test "$(rospack find reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/reference_trajectory"
test "$(rospack find multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_controller"
test "$(rospack find estimator_rigid_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_rigid_state"
test -f "/opt/ros/${ROS_DISTRO}/share/reference_trajectory/config/reference_trajectory.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/reference_trajectory/launch/uav_reference_trajectory.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/core/trajectory_core.h"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/AnalyticReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/WaypointReferenceRequest.h"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/SampledReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/ActivePolynomialReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/reference_trajectory/ReferenceStatus.h"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_controller/config/uav_nmpc.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_controller/launch/uav_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/drone_controller.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/uav/state_machine/custom1_state.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_controller/SetRuntimeParameters.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/multirotor_controller/multirotor_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_controller_uav_nmpc_runtime.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libreference_trajectory_core.so"
roslaunch --files reference_trajectory uav_reference_trajectory.launch >/tmp/xgc2-reference-files.txt
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
  "/opt/ros/${ROS_DISTRO}/lib/reference_trajectory" \
  "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_controller_uav_nmpc_runtime.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libreference_trajectory_core.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
