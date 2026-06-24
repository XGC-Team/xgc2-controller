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
test "$(rospack find multirotor_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_reference_trajectory"
test "$(rospack find px4_multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/px4_multirotor_controller"
test "$(rospack find unicycle_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller"
test "$(rospack find estimator_vrpn_px4_rotor_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_px4_rotor_state"
test "$(rospack find estimator_vrpn_ugv_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_reference_trajectory/config/reference_trajectory.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/multirotor_reference_trajectory/launch/uav_multirotor_reference_trajectory.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/core/trajectory_core.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/AnalyticReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/WaypointReferenceRequest.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/SampledReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/ActivePolynomialReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/multirotor_reference_trajectory/ReferenceStatus.h"
test -f "/opt/ros/${ROS_DISTRO}/share/px4_multirotor_controller/config/uav_nmpc.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/px4_multirotor_controller/launch/uav_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/px4_multirotor_controller/drone_controller.h"
test -f "/opt/ros/${ROS_DISTRO}/include/px4_multirotor_controller/uav/state_machine/custom1_state.h"
test -f "/opt/ros/${ROS_DISTRO}/include/px4_multirotor_controller/SetRuntimeParameters.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/px4_multirotor_controller/px4_multirotor_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/config/unicycle_reference_trajectory.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/launch/ugv_unicycle_reference_trajectory.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/core/trajectory_core.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/AnalyticReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/PlanarReferencePoint.h"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/config/unicycle_ugv_controller.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/launch/ugv_unicycle_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_ugv_controller/unicycle_ugv_controller.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller/unicycle_ugv_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/lib/libpx4_multirotor_controller_uav_nmpc_runtime.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_reference_trajectory_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libunicycle_reference_trajectory_core.so"
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch >/tmp/xgc2-multirotor-reference-files.txt
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch >/tmp/xgc2-px4-controller-files.txt
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/px4_multirotor_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/multirotor_reference_trajectory" \
  "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/unicycle_reference_trajectory" \
  "/opt/ros/${ROS_DISTRO}/lib/libpx4_multirotor_controller_uav_nmpc_runtime.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libmultirotor_reference_trajectory_core.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so" \
  "/opt/ros/${ROS_DISTRO}/lib/libunicycle_reference_trajectory_core.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
