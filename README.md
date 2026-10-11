# xgc2-controller

Source and compatibility catalog for the XGC2 ROS1 controller products. It does
not publish an aggregate Debian package.

The controller products are pinned as submodules; each child repository owns and
releases its own package:

- `multirotor-controller` -> `XGC-Team/xgc2-multirotor-controller`
  (`ros-noetic-xgc2-multirotor-controller`)
- `ugv-controller` -> `XGC-Team/xgc2-ugv-controller`
  (`ros-noetic-xgc2-ugv-controller`)

Consumers install only the products they use:

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-multirotor-controller ros-noetic-xgc2-ugv-controller
```

`.xgc2/release-set.yml` records the compatible product versions for auditing. The
former aggregate package `ros-noetic-xgc2-controller` had no consumer and is
retired from the APT index.

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
```
