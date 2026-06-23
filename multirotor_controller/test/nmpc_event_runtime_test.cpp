#include "multirotor_controller/uav/nmpc_result_buffer.h"
#include "multirotor_controller/uav/reference_trajectory_buffer.h"
#include "multirotor_controller/nmpc/uav_nmpc_solver.h"

#include <gtest/gtest.h>

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

TEST(ReferenceTrajectoryBuffer, FlatTrajectorySamplesAndBuildsHorizon) {
  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(buffer.updateFlat(makeFlatTrajectory(), ros::Time(10.0)));

  UavReferencePoint sample;
  ASSERT_TRUE(buffer.sample(ros::Time(10.5), 1.0, sample));
  EXPECT_NEAR(sample.position.x(), 0.5, 1e-9);
  EXPECT_NEAR(sample.position.y(), 1.0, 1e-9);
  EXPECT_EQ(buffer.sequence(), 7U);

  std::vector<reference_trajectory::UavReferenceSample> horizon;
  ASSERT_TRUE(buffer.sampleHorizon(ros::Time(10.0), 0.02, 20, 1.0, 9.8066,
                                   horizon));
  EXPECT_EQ(horizon.size(), 22U);
  EXPECT_TRUE(horizon.front().x.array().isFinite().all());
  EXPECT_TRUE(horizon.front().u.array().isFinite().all());
}

TEST(ReferenceTrajectoryBuffer, BsplineTrajectorySamplesFiniteValues) {
  ReferenceTrajectoryBuffer buffer;
  ASSERT_TRUE(buffer.updateBspline(makeBsplineTrajectory(), ros::Time(20.0),
                                   0.05));

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

} // namespace
} // namespace multirotor_controller
