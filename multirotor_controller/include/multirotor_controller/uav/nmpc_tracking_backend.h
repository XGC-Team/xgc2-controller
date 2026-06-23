#pragma once

#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "reference_trajectory/nmpc_reference_trajectory.h"

namespace multirotor_controller {

using reference_trajectory::UavReferenceSample;

class UavNmpcTrackingBackend {
public:
  void configure(const ControllerConfig &config);
  bool enter(const SensorData &sensor);
  bool compute(const SensorData &sensor, const MpcTrajectoryState &reference,
               const ros::Time &now, AttitudeRateTarget &target);
  bool compute(
      const SensorData &sensor,
      const std::vector<reference_trajectory::UavReferenceSample> &references,
      const ros::Time &now, AttitudeRateTarget &target);
  void exit();

  int status() const { return solver_.status(); }
  double solveTimeMs() const { return solver_.solveTimeMs(); }

private:
  bool feedbackState(const SensorData &sensor, Vector13d &x0) const;
  std::vector<UavReferenceSample>
  buildReferenceHorizon(const MpcTrajectoryState &reference,
                        const ros::Time &now) const;
  UavReferenceSample sampleReference(const MpcTrajectoryState &reference,
                                     double dt) const;
  bool hoverThrustReady(const SensorData &sensor, const ros::Time &now) const;
  double mapSpecificThrustToNormalized(double specific_thrust,
                                       double hover_thrust) const;

  ControllerConfig config_{};
  UavNmpcSolver solver_;

  bool entered_{false};

  ros::Time last_control_time_;
  ros::Time last_log_time_;
};

} // namespace multirotor_controller
