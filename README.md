# XGC2 Controller

ROS1 controller product repository for XGC2 robots.

Current packages:

- `multirotor_controller`: PX4/MAVROS multirotor controller with state-machine
  runtime and UAV NMPC tracking.

Future controller packages, such as UGV controllers, should be added as sibling
ROS packages under this repository root.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-controller
```

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_controller uav_nmpc_controller.launch
```
