#pragma once

#include "multirotor_controller/common/types.h"

#include <cmath>

namespace multirotor_controller {
namespace sensor_checks {

constexpr double kAirborneAltitudeThreshold = 0.3;

inline bool areBaseSensorsActive(const SensorData &sensor) {
  return sensor.local_pos_stats.is_active &&
         sensor.local_velocity_stats.is_active && sensor.imu_stats.is_active &&
         sensor.state_stats.is_active && sensor.battery_stats.is_active;
}

inline bool areVrpnTopicsActive(const SensorData &sensor) {
  return sensor.vrpn_pose_stats.is_active && sensor.vrpn_twist_stats.is_active;
}

inline double vrpnLocalPositionDiff(const SensorData &sensor) {
  const double dx = sensor.vrpn_x - sensor.x;
  const double dy = sensor.vrpn_y - sensor.y;
  const double dz = sensor.vrpn_z - sensor.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline bool isVrpnPoseConsistent(const SensorData &sensor,
                                 double position_tolerance = 0.05) {
  return vrpnLocalPositionDiff(sensor) < position_tolerance;
}

inline bool areSensorsAllActive(const SensorData &sensor) {
  return areBaseSensorsActive(sensor) && areVrpnTopicsActive(sensor) &&
         isVrpnPoseConsistent(sensor);
}

inline bool isFcuConnected(const SensorData &sensor) {
  return sensor.fcu_connected;
}

inline bool isFcuArmed(const SensorData &sensor) { return sensor.fcu_armed; }

inline bool hasManualInput(const SensorData &sensor) {
  return sensor.fcu_manual_input;
}

inline bool isOffboardMode(const SensorData &sensor) {
  return sensor.fcu_mode == "OFFBOARD";
}

inline bool isAirborne(const SensorData &sensor) {
  return sensor.fcu_armed && sensor.z > kAirborneAltitudeThreshold;
}

} // namespace sensor_checks
} // namespace multirotor_controller
