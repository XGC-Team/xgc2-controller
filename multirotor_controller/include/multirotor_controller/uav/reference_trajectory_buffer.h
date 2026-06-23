#pragma once

#include "multirotor_controller/common/types.h"
#include "multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "reference_trajectory/UavBsplineTrajectory.h"
#include "reference_trajectory/UavFlatTrajectory.h"
#include "reference_trajectory/nmpc_reference_trajectory.h"

#include <Eigen/Dense>
#include <mutex>
#include <vector>

namespace multirotor_controller {

struct UavReferencePoint {
  double t_from_start{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_accel{0.0};
};

class ReferenceTrajectoryBuffer {
public:
  bool updateFlat(const reference_trajectory::UavFlatTrajectory &msg,
                  const ros::Time &received_time);
  bool updateBspline(const reference_trajectory::UavBsplineTrajectory &msg,
                     const ros::Time &received_time, double sample_dt);
  bool updateLegacySetpoint(const MpcTrajectoryState &trajectory,
                            const ros::Time &received_time);
  void clear();

  bool sample(const ros::Time &now, double timeout,
              UavReferencePoint &sample) const;
  bool sampleHorizon(const ros::Time &now, double stage_dt, int horizon_steps,
                     double timeout, double gravity,
                     std::vector<reference_trajectory::UavReferenceSample>
                         &references) const;

  uint64_t sequence() const;
  bool valid(const ros::Time &now, double timeout) const;

private:
  static bool finite(const UavReferencePoint &point);
  static bool finiteVector(const Eigen::Vector3d &value);
  static double normalizeTime(double value);
  static bool interpolate(const std::vector<UavReferencePoint> &points,
                          double t, UavReferencePoint &sample);
  static bool evaluateBspline(const std::vector<double> &knots,
                              const std::vector<double> &values, int degree,
                              double u, double &value);
  static bool evaluateBsplinePoint(const std::vector<double> &knots,
                                   const std::vector<Eigen::Vector3d> &points,
                                   int degree, double u,
                                   Eigen::Vector3d &value);

  mutable std::mutex mutex_;
  uint64_t trajectory_id_{0};
  uint64_t sequence_{0};
  ros::Time start_time_;
  ros::Time received_time_;
  bool valid_{false};
  std::vector<UavReferencePoint> points_;
};

} // namespace multirotor_controller
