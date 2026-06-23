#include <gtest/gtest.h>

#include <cmath>

#include "multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "multirotor_controller/uav/nmpc_result_buffer.h"
#include "multirotor_controller/uav/reference_trajectory_buffer.h"

namespace multirotor_controller {
namespace {

reference_trajectory::UavFlatTrajectory makeFlatTrajectory() {
  reference_trajectory::UavFlatTrajectory msg;
  msg.valid = true;
  msg.sequence = 7;
  msg.start_time = ros::Time(10.0);
  for (int i = 0; i < 2; ++i) {
    reference_trajectory::UavFlatTrajectoryPoint point;
    point.t_from_start = static_cast<double>(i);
    point.position.x = static_cast<double>(i);
    point.position.y = 2.0 * static_cast<double>(i);
    point.position.z = 1.0;
    point.velocity.x = 1.0;
    point.velocity.y = 2.0;
    msg.points.push_back(point);
  }
  return msg;
}

reference_trajectory::UavBsplineTrajectory makeBsplineTrajectory() {
  reference_trajectory::UavBsplineTrajectory msg;
  msg.valid = true;
  msg.sequence = 8;
  msg.start_time = ros::Time(20.0);
  msg.order = 3;
  msg.knots = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::Point point;
    point.x = static_cast<double>(i);
    point.y = 0.0;
    point.z = 1.0;
    msg.position_control_points.push_back(point);
    msg.yaw_control_points.push_back(0.0);
  }
  return msg;
}

reference_trajectory::UavFlatTrajectory makeCircleFlatTrajectory() {
  constexpr double kRadius = 1.0;
  constexpr double kOmega = 0.35;
  constexpr double kHeight = 3.0;
  constexpr double kSampleDt = 0.01;
  constexpr int kSampleCount = 2200;

  reference_trajectory::UavFlatTrajectory msg;
  msg.valid = true;
  msg.sequence = 10;
  msg.start_time = ros::Time(100.0);
  for (int i = 0; i < kSampleCount; ++i) {
    const double t = static_cast<double>(i) * kSampleDt;
    const double wt = kOmega * t;
    const double s = std::sin(wt);
    const double c = std::cos(wt);

    reference_trajectory::UavFlatTrajectoryPoint point;
    point.t_from_start = t;
    point.position.x = kRadius * c;
    point.position.y = kRadius * s;
    point.position.z = kHeight;
    point.velocity.x = -kRadius * kOmega * s;
    point.velocity.y = kRadius * kOmega * c;
    point.acceleration.x = -kRadius * kOmega * kOmega * c;
    point.acceleration.y = -kRadius * kOmega * kOmega * s;
    point.jerk.x = kRadius * kOmega * kOmega * kOmega * s;
    point.jerk.y = -kRadius * kOmega * kOmega * kOmega * c;
    point.snap.x = kRadius * std::pow(kOmega, 4) * c;
    point.snap.y = kRadius * std::pow(kOmega, 4) * s;
    point.yaw = M_PI / 2.0 + wt;
    point.yaw_rate = kOmega;
    point.yaw_accel = 0.0;
    msg.points.push_back(point);
  }
  return msg;
}

TEST(ReferenceTrajectoryBuffer, FlatTrajectorySamplesAndBuildsHorizon) {
  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(buffer.updateFlat(makeFlatTrajectory(), ros::Time(10.0)));

  UavReferencePoint sample;
  ASSERT_TRUE(buffer.sample(ros::Time(10.5), 1.0, sample));
  EXPECT_NEAR(sample.position.x(), 0.5, 1e-9);
  EXPECT_NEAR(sample.position.y(), 1.0, 1e-9);
  EXPECT_EQ(buffer.sequence(), 7U);

  std::vector<reference_trajectory::UavReferenceSample> horizon;
  ASSERT_TRUE(
      buffer.sampleHorizon(ros::Time(10.0), 0.02, 20, 1.0, 9.8066, horizon));
  EXPECT_EQ(horizon.size(), 22U);
  EXPECT_TRUE(horizon.front().x.array().isFinite().all());
  EXPECT_TRUE(horizon.front().u.array().isFinite().all());
}

TEST(ReferenceTrajectoryBuffer, FlatTrajectoryUnwrapsYawAcrossPiBoundary) {
  reference_trajectory::UavFlatTrajectory msg;
  msg.valid = true;
  msg.sequence = 9;
  msg.start_time = ros::Time(30.0);
  for (int i = 0; i < 2; ++i) {
    reference_trajectory::UavFlatTrajectoryPoint point;
    point.t_from_start = static_cast<double>(i);
    point.position.z = 1.0;
    point.yaw = i == 0 ? 3.13 : -3.13;
    msg.points.push_back(point);
  }

  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(buffer.updateFlat(msg, ros::Time(30.0)));

  UavReferencePoint sample;
  ASSERT_TRUE(buffer.sample(ros::Time(30.5), 1.0, sample));
  EXPECT_NEAR(sample.yaw, M_PI, 2e-3);

  std::vector<reference_trajectory::UavReferenceSample> horizon;
  ASSERT_TRUE(
      buffer.sampleHorizon(ros::Time(30.0), 0.1, 10, 1.0, 9.8066, horizon));
  for (const auto& reference : horizon) {
    EXPECT_TRUE(reference.x.array().isFinite().all());
    EXPECT_TRUE(reference.u.array().isFinite().all());
    EXPECT_LT(reference.x.segment<3>(10).norm(), 1.0);
    EXPECT_LT(reference.u.tail<3>().norm(), 1.0);
  }
}

TEST(ReferenceTrajectoryBuffer, BsplineTrajectorySamplesFiniteValues) {
  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(
      buffer.updateBspline(makeBsplineTrajectory(), ros::Time(20.0), 0.05));

  UavReferencePoint sample;
  ASSERT_TRUE(buffer.sample(ros::Time(20.5), 1.0, sample));
  EXPECT_TRUE(sample.position.array().isFinite().all());
  EXPECT_TRUE(sample.velocity.array().isFinite().all());
}

TEST(NmpcResultBuffer, KeepsNewestSequence) {
  NmpcResultBuffer buffer;
  NmpcSolveResult newer;
  newer.sequence = 2;
  newer.success = true;
  newer.stamp = ros::Time(1.0);
  buffer.store(newer);

  NmpcSolveResult older;
  older.sequence = 1;
  older.success = false;
  older.stamp = ros::Time(2.0);
  buffer.store(older);

  NmpcSolveResult output;
  ASSERT_TRUE(buffer.consumeNewerThan(0, output));
  EXPECT_EQ(output.sequence, 2U);
  EXPECT_TRUE(output.success);
  EXPECT_TRUE(buffer.hasFreshSuccess(ros::Time(1.05), 0.1));
  EXPECT_FALSE(buffer.hasFreshSuccess(ros::Time(1.2), 0.1));
}

TEST(UavNmpcSolver, SolvesHoverEquilibrium) {
  ros::Time::init();
  UavNmpcSolver solver;
  ASSERT_TRUE(solver.initialize());

  reference_trajectory::UavReferenceSample hover;
  hover.x.setZero();
  hover.x(2) = 1.0;
  hover.x(6) = 1.0;
  hover.u << 9.8066, 0.0, 0.0, 0.0;

  Vector13d x0 = hover.x;
  std::vector<reference_trajectory::UavReferenceSample> references(
      static_cast<size_t>(UavNmpcSolver::horizonSteps() + 2), hover);
  EXPECT_TRUE(solver.solve(x0, references)) << "status=" << solver.status();
  EXPECT_NEAR(solver.optimalControl()(0), 9.8066, 1e-3);
  EXPECT_NEAR(solver.predictedBodyRate().norm(), 0.0, 1e-3);
}

TEST(UavNmpcSolver, CircleReferenceSmallErrorsDoNotBangAngularAcceleration) {
  ros::Time::init();
  UavNmpcSolver solver;
  ASSERT_TRUE(solver.initialize());

  reference_trajectory::UavReferenceConfig config;
  config.omega = 0.35;
  config.gravity = 9.8066;

  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(buffer.updateFlat(makeCircleFlatTrajectory(), ros::Time(100.0)));

  constexpr double kStageDt = 0.1;
  constexpr double kSaturationGuard = 8.5;
  int near_saturation_count = 0;
  double max_angular_accel = 0.0;
  for (int phase = 0; phase < 360; phase += 20) {
    const double t0 =
        (static_cast<double>(phase) * M_PI / 180.0) / config.omega;
    std::vector<reference_trajectory::UavReferenceSample> references;
    ASSERT_TRUE(buffer.sampleHorizon(ros::Time(100.0 + t0), kStageDt,
                                     UavNmpcSolver::horizonSteps(), 100.0,
                                     config.gravity, references));

    Vector13d x0 = references.front().x;
    const double angle = static_cast<double>(phase) * M_PI / 180.0;
    x0(0) += 0.01 * std::cos(angle);
    x0(1) += 0.01 * std::sin(angle);
    x0(3) += 0.02 * -std::sin(angle);
    x0(4) += 0.02 * std::cos(angle);

    solver.resetWarmStart();
    EXPECT_TRUE(solver.solve(x0, references))
        << "phase=" << phase << " status=" << solver.status();
    const Vector4d control = solver.optimalControl();
    const double angular_accel = control.tail<3>().cwiseAbs().maxCoeff();
    max_angular_accel = std::max(max_angular_accel, angular_accel);
    if (angular_accel > kSaturationGuard) {
      ++near_saturation_count;
    }
  }

  EXPECT_LE(max_angular_accel, kSaturationGuard);
  EXPECT_EQ(near_saturation_count, 0);
}

}  // namespace
}  // namespace multirotor_controller
