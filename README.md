# XGC2 Controller

ROS1 controller product repository for XGC2 robots.

Current packages:

- `px4_multirotor_controller`: PX4/MAVROS multirotor controller with state-machine
  runtime and UAV NMPC tracking.
- `multirotor_reference_trajectory`: reference trajectory messages, generation library, and
  ROS publishers used by the multirotor controller.
- `unicycle_ugv_controller`: unicycle-model UGV NMPC controller publishing
  `geometry_msgs/Twist`.
- `unicycle_reference_trajectory`: planar analytic, waypoint, and sampled
  reference trajectory messages and publishers for unicycle UGV tracking.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-controller
```

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
```
