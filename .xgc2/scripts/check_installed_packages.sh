#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-multirotor-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ugv-controller" >/dev/null

test "$(rospack find multirotor_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/multirotor_reference_trajectory"
test "$(rospack find px4_multirotor_controller)" = "/opt/ros/${ROS_DISTRO}/share/px4_multirotor_controller"
test "$(rospack find unicycle_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller"

roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch >/tmp/xgc2-multirotor-reference-files.txt
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch >/tmp/xgc2-px4-controller-files.txt
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt

echo "Installed aggregate package check passed"
