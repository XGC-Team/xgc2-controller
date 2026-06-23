#include "multirotor_controller/uav/reference_trajectory_buffer.h"

#include <algorithm>
#include <cmath>

#include "multirotor_controller/nmpc/nmpc_math_utils.h"

namespace multirotor_controller {

namespace {

Eigen::Vector3d toVector(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d toVector(const geometry_msgs::Vector3& vector) {
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

double wrapAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

double unwrapAngleNear(double value, double reference) {
  return reference + wrapAngle(value - reference);
}

Eigen::Vector3d unitVectorDerivative(const Eigen::Vector3d& value,
                                     const Eigen::Vector3d& value_dot,
                                     const Eigen::Vector3d& unit_value) {
  const double norm = value.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  return (Eigen::Matrix3d::Identity() - unit_value * unit_value.transpose()) *
         value_dot / norm;
}

struct FlatKinematics {
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d omega{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust{Eigen::Vector3d::Zero()};
};

FlatKinematics flatKinematics(const UavReferencePoint& point, double gravity) {
  FlatKinematics out;
  out.thrust = point.acceleration + gravity * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d b3 =
      normalizeVector(out.thrust, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d b3_dot =
      unitVectorDerivative(out.thrust, point.jerk, b3);

  const double yaw = point.yaw;
  const double yaw_rate = point.yaw_rate;
  const Eigen::Vector3d xc(std::cos(yaw), std::sin(yaw), 0.0);
  const Eigen::Vector3d xc_dot(-std::sin(yaw) * yaw_rate,
                               std::cos(yaw) * yaw_rate, 0.0);

  Eigen::Vector3d y_raw = b3.cross(xc);
  Eigen::Vector3d y_raw_dot = b3_dot.cross(xc) + b3.cross(xc_dot);
  if (!std::isfinite(y_raw.norm()) || y_raw.norm() < 1e-8) {
    const Eigen::Vector3d fallback =
        std::abs(b3.dot(Eigen::Vector3d::UnitY())) > 0.95
            ? Eigen::Vector3d::UnitX()
            : Eigen::Vector3d::UnitY();
    y_raw = b3.cross(fallback);
    y_raw_dot.setZero();
  }

  const Eigen::Vector3d yb = normalizeVector(y_raw, Eigen::Vector3d::UnitY());
  const Eigen::Vector3d yb_dot = unitVectorDerivative(y_raw, y_raw_dot, yb);
  const Eigen::Vector3d xb =
      normalizeVector(yb.cross(b3), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d xb_dot = yb_dot.cross(b3) + yb.cross(b3_dot);

  out.rotation.col(0) = xb;
  out.rotation.col(1) = yb;
  out.rotation.col(2) = b3;
  out.rotation = projectRotation(out.rotation);

  Eigen::Matrix3d rotation_dot;
  rotation_dot.col(0) = xb_dot;
  rotation_dot.col(1) = yb_dot;
  rotation_dot.col(2) = b3_dot;
  const Eigen::Matrix3d omega_hat = out.rotation.transpose() * rotation_dot;
  out.omega = vee(0.5 * (omega_hat - omega_hat.transpose()));
  if (!out.omega.array().isFinite().all()) {
    out.omega.setZero();
  }
  return out;
}

}  // namespace

bool ReferenceTrajectoryBuffer::updateFlat(
    const reference_trajectory::UavFlatTrajectory& msg,
    const ros::Time& received_time) {
  if (!msg.valid || msg.points.empty()) {
    return false;
  }

  std::vector<UavReferencePoint> points;
  points.reserve(msg.points.size());
  double last_t = -1.0;
  for (const auto& input : msg.points) {
    UavReferencePoint point;
    point.t_from_start = normalizeTime(input.t_from_start);
    if (point.t_from_start < last_t) {
      return false;
    }
    last_t = point.t_from_start;
    point.position = toVector(input.position);
    point.velocity = toVector(input.velocity);
    point.acceleration = toVector(input.acceleration);
    point.jerk = toVector(input.jerk);
    point.snap = toVector(input.snap);
    point.yaw = input.yaw;
    if (!points.empty()) {
      point.yaw = unwrapAngleNear(point.yaw, points.back().yaw);
    }
    point.yaw_rate = input.yaw_rate;
    point.yaw_accel = input.yaw_accel;
    if (!finite(point)) {
      return false;
    }
    points.push_back(point);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_id_ = msg.trajectory_id;
  sequence_ = msg.sequence;
  start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
  if (start_time_.isZero()) {
    start_time_ = received_time;
  }
  received_time_ = received_time;
  points_ = std::move(points);
  valid_ = true;
  return true;
}

bool ReferenceTrajectoryBuffer::updateBspline(
    const reference_trajectory::UavBsplineTrajectory& msg,
    const ros::Time& received_time, double sample_dt) {
  const int degree = std::max(1, msg.order);
  if (!msg.valid || msg.knots.size() < static_cast<size_t>(degree + 2) ||
      msg.position_control_points.empty() || sample_dt <= 1e-4) {
    return false;
  }

  std::vector<Eigen::Vector3d> control_points;
  control_points.reserve(msg.position_control_points.size());
  for (const auto& point : msg.position_control_points) {
    const auto value = toVector(point);
    if (!finiteVector(value)) {
      return false;
    }
    control_points.push_back(value);
  }

  const double u_min = msg.knots[static_cast<size_t>(degree)];
  const double u_max =
      msg.knots[msg.knots.size() - static_cast<size_t>(degree) - 1U];
  if (!std::isfinite(u_min) || !std::isfinite(u_max) || u_max <= u_min) {
    return false;
  }

  const int sample_count =
      std::max(2, static_cast<int>(std::ceil((u_max - u_min) / sample_dt)) + 1);
  std::vector<double> yaw_control_points = msg.yaw_control_points;
  for (size_t i = 1; i < yaw_control_points.size(); ++i) {
    yaw_control_points[i] =
        unwrapAngleNear(yaw_control_points[i], yaw_control_points[i - 1]);
  }

  std::vector<UavReferencePoint> points(static_cast<size_t>(sample_count));
  for (int i = 0; i < sample_count; ++i) {
    const double t =
        std::min(u_max - u_min, static_cast<double>(i) * sample_dt);
    const double u = u_min + t;
    auto& point = points[static_cast<size_t>(i)];
    point.t_from_start = t;
    if (!evaluateBsplinePoint(msg.knots, control_points, degree, u,
                              point.position)) {
      return false;
    }
    if (!yaw_control_points.empty()) {
      (void)evaluateBspline(msg.knots, yaw_control_points, degree, u,
                            point.yaw);
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t prev = i == 0 ? i : i - 1;
    const size_t next = std::min(points.size() - 1, i + 1);
    const double dt =
        std::max(1e-6, points[next].t_from_start - points[prev].t_from_start);
    points[i].velocity = (points[next].position - points[prev].position) / dt;
    points[i].yaw_rate = wrapAngle(points[next].yaw - points[prev].yaw) / dt;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    const size_t prev = i == 0 ? i : i - 1;
    const size_t next = std::min(points.size() - 1, i + 1);
    const double dt =
        std::max(1e-6, points[next].t_from_start - points[prev].t_from_start);
    points[i].acceleration =
        (points[next].velocity - points[prev].velocity) / dt;
    points[i].yaw_accel = (points[next].yaw_rate - points[prev].yaw_rate) / dt;
    if (!finite(points[i])) {
      return false;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_id_ = msg.trajectory_id;
  sequence_ = msg.sequence;
  start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
  if (start_time_.isZero()) {
    start_time_ = received_time;
  }
  received_time_ = received_time;
  points_ = std::move(points);
  valid_ = true;
  return true;
}

bool ReferenceTrajectoryBuffer::updateLegacySetpoint(
    const MpcTrajectoryState& trajectory, const ros::Time& received_time) {
  if (!trajectory.is_valid) {
    return false;
  }
  UavReferencePoint point;
  point.t_from_start = 0.0;
  point.position = trajectory.position_k;
  point.velocity = trajectory.velocity_k;
  point.acceleration = trajectory.acceleration_k;
  if (!finite(point)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  ++sequence_;
  start_time_ = trajectory.planning_time.isZero() ? received_time
                                                  : trajectory.planning_time;
  received_time_ = received_time;
  points_.assign(1, point);
  valid_ = true;
  return true;
}

void ReferenceTrajectoryBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_id_ = 0;
  sequence_ = 0;
  start_time_ = ros::Time();
  received_time_ = ros::Time();
  valid_ = false;
  points_.clear();
}

bool ReferenceTrajectoryBuffer::sample(const ros::Time& now, double timeout,
                                       UavReferencePoint& sample) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_ || points_.empty() || start_time_.isZero() ||
      (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
    return false;
  }
  const double t = std::max(0.0, (now - start_time_).toSec());
  return interpolate(points_, t, sample);
}

bool ReferenceTrajectoryBuffer::sampleHorizon(
    const ros::Time& now, double stage_dt, int horizon_steps, double timeout,
    double gravity,
    std::vector<reference_trajectory::UavReferenceSample>& references) const {
  if (horizon_steps <= 0 || stage_dt <= 0.0) {
    return false;
  }

  std::vector<UavReferencePoint> local_points;
  ros::Time local_start;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid_ || points_.empty() || start_time_.isZero() ||
        (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
      return false;
    }
    local_points = points_;
    local_start = start_time_;
  }

  const std::size_t reference_count =
      static_cast<std::size_t>(horizon_steps + 2);
  const double derivative_dt = std::max(1e-3, std::min(0.01, 0.25 * stage_dt));
  references.clear();
  references.reserve(reference_count);

  auto sample_point = [&](double t, UavReferencePoint& point) {
    return interpolate(local_points, std::max(0.0, t), point);
  };

  auto omega_at = [&](double t, Eigen::Vector3d& omega) {
    UavReferencePoint point;
    if (!sample_point(t, point)) {
      return false;
    }
    omega = flatKinematics(point, gravity).omega;
    return omega.array().isFinite().all();
  };

  auto alpha_at = [&](double t, Eigen::Vector3d& alpha) {
    Eigen::Vector3d omega_minus;
    Eigen::Vector3d omega_plus;
    if (t <= derivative_dt) {
      Eigen::Vector3d omega_now;
      if (!omega_at(t, omega_now) || !omega_at(t + derivative_dt, omega_plus)) {
        return false;
      }
      alpha = (omega_plus - omega_now) / derivative_dt;
      return true;
    }
    if (!omega_at(t - derivative_dt, omega_minus) ||
        !omega_at(t + derivative_dt, omega_plus)) {
      return false;
    }
    alpha = (omega_plus - omega_minus) / (2.0 * derivative_dt);
    return alpha.array().isFinite().all();
  };

  for (int i = 0; i <= horizon_steps + 1; ++i) {
    UavReferencePoint point;
    const double t = std::max(
        0.0, (now + ros::Duration(i * stage_dt) - local_start).toSec());
    if (!sample_point(t, point)) {
      return false;
    }
    const FlatKinematics kinematics = flatKinematics(point, gravity);
    Eigen::Vector3d alpha;
    if (!alpha_at(t, alpha)) {
      return false;
    }

    reference_trajectory::UavReferenceSample sample;
    sample.x.setZero();
    sample.x.segment<3>(0) = point.position;
    sample.x.segment<3>(3) = point.velocity;
    sample.x.segment<4>(6) =
        quatToVecWxyz(Eigen::Quaterniond(kinematics.rotation));
    sample.x.segment<3>(10) = kinematics.omega;
    sample.u << kinematics.thrust.norm(), alpha;
    references.push_back(sample);
  }
  return true;
}

uint64_t ReferenceTrajectoryBuffer::sequence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return sequence_;
}

bool ReferenceTrajectoryBuffer::valid(const ros::Time& now,
                                      double timeout) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return valid_ && !points_.empty() &&
         (timeout <= 0.0 || (now - received_time_).toSec() <= timeout);
}

bool ReferenceTrajectoryBuffer::finite(const UavReferencePoint& point) {
  return std::isfinite(point.t_from_start) && finiteVector(point.position) &&
         finiteVector(point.velocity) && finiteVector(point.acceleration) &&
         finiteVector(point.jerk) && finiteVector(point.snap) &&
         std::isfinite(point.yaw) && std::isfinite(point.yaw_rate) &&
         std::isfinite(point.yaw_accel);
}

bool ReferenceTrajectoryBuffer::finiteVector(const Eigen::Vector3d& value) {
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}

double ReferenceTrajectoryBuffer::normalizeTime(double value) {
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::max(0.0, value);
}

bool ReferenceTrajectoryBuffer::interpolate(
    const std::vector<UavReferencePoint>& points, double t,
    UavReferencePoint& sample) {
  if (points.empty() || !std::isfinite(t)) {
    return false;
  }
  if (points.size() == 1 || t <= points.front().t_from_start) {
    sample = points.front();
    return true;
  }
  if (t >= points.back().t_from_start) {
    sample = points.back();
    return true;
  }

  const auto upper =
      std::upper_bound(points.begin(), points.end(), t,
                       [](double lhs, const UavReferencePoint& rhs) {
                         return lhs < rhs.t_from_start;
                       });
  const auto next = upper;
  const auto prev = upper - 1;
  const double dt = next->t_from_start - prev->t_from_start;
  if (dt <= 1e-9) {
    sample = *prev;
    return true;
  }
  const double alpha = (t - prev->t_from_start) / dt;
  sample.t_from_start = t;
  sample.position = (1.0 - alpha) * prev->position + alpha * next->position;
  sample.velocity = (1.0 - alpha) * prev->velocity + alpha * next->velocity;
  sample.acceleration =
      (1.0 - alpha) * prev->acceleration + alpha * next->acceleration;
  sample.jerk = (1.0 - alpha) * prev->jerk + alpha * next->jerk;
  sample.snap = (1.0 - alpha) * prev->snap + alpha * next->snap;
  sample.yaw = prev->yaw + alpha * wrapAngle(next->yaw - prev->yaw);
  sample.yaw_rate = (1.0 - alpha) * prev->yaw_rate + alpha * next->yaw_rate;
  sample.yaw_accel = (1.0 - alpha) * prev->yaw_accel + alpha * next->yaw_accel;
  return finite(sample);
}

bool ReferenceTrajectoryBuffer::evaluateBspline(
    const std::vector<double>& knots, const std::vector<double>& values,
    int degree, double u, double& value) {
  if (degree < 1 ||
      values.size() + static_cast<size_t>(degree) + 1U != knots.size()) {
    return false;
  }
  const int n = static_cast<int>(values.size()) - 1;
  int k = degree;
  for (int i = degree; i <= n; ++i) {
    if (u >= knots[static_cast<size_t>(i)] &&
        u <= knots[static_cast<size_t>(i + 1)]) {
      k = i;
      break;
    }
  }
  if (u >= knots[static_cast<size_t>(n + 1)]) {
    k = n;
  }

  std::vector<double> d(static_cast<size_t>(degree + 1));
  for (int j = 0; j <= degree; ++j) {
    d[static_cast<size_t>(j)] = values[static_cast<size_t>(k - degree + j)];
  }
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const int idx = k - degree + j;
      const double denom = knots[static_cast<size_t>(idx + degree + 1 - r)] -
                           knots[static_cast<size_t>(idx)];
      const double alpha = std::abs(denom) < 1e-9
                               ? 0.0
                               : (u - knots[static_cast<size_t>(idx)]) / denom;
      d[static_cast<size_t>(j)] =
          (1.0 - alpha) * d[static_cast<size_t>(j - 1)] +
          alpha * d[static_cast<size_t>(j)];
    }
  }
  value = d[static_cast<size_t>(degree)];
  return std::isfinite(value);
}

bool ReferenceTrajectoryBuffer::evaluateBsplinePoint(
    const std::vector<double>& knots,
    const std::vector<Eigen::Vector3d>& points, int degree, double u,
    Eigen::Vector3d& value) {
  std::vector<double> axis(points.size());
  for (int dim = 0; dim < 3; ++dim) {
    for (size_t i = 0; i < points.size(); ++i) {
      axis[i] = points[i](dim);
    }
    if (!evaluateBspline(knots, axis, degree, u, value(dim))) {
      return false;
    }
  }
  return finiteVector(value);
}

}  // namespace multirotor_controller
