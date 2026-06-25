# XGC2 Controller

Aggregate repository for XGC2 ROS1 controllers.

This repository no longer owns controller source code directly. It pins the split
controller products as submodules and publishes `ros-noetic-xgc2-controller` as a
small metapackage:

- `multirotor-controller` -> `lxk36/xgc2-multirotor-controller`
- `ugv-controller` -> `lxk36/xgc2-ugv-controller`

Install the aggregate package when a deployment needs both controller families:

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-controller
```

The concrete ROS packages are provided by the split products:

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
```
