#include <gtest/gtest.h>

#include <cmath>

#include "multirotor_reference_trajectory/core/trajectory_core.h"

namespace {

TEST(ReferenceTrajectoryCore, HeightCircleProvidesHighOrderDerivatives) {
    multirotor_reference_trajectory::core::AnalyticParameters params;
    params.type = multirotor_reference_trajectory::core::AnalyticType::kHeightCircle;
    params.radius = 3.0;
    params.line_speed = 3.0;
    params.height = 3.0;
    params.z_amplitude = 1.0;
    params.z_frequency = 0.5;

    multirotor_reference_trajectory::core::AnalyticEvaluator evaluator(params);
    multirotor_reference_trajectory::core::FlatOutput output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.velocity.y(), 3.0, 1e-12);
    EXPECT_TRUE(output.snap.allFinite());
    EXPECT_TRUE(std::isfinite(output.yaw_rate));
    EXPECT_TRUE(std::isfinite(output.yaw_accel));
}

TEST(ReferenceTrajectoryCore, WaypointSolverProducesMinSnapPolynomial) {
    multirotor_reference_trajectory::core::WaypointProblem problem;
    for (const auto& point : {Eigen::Vector3d(0.0, 0.0, 3.0), Eigen::Vector3d(1.0, 1.0, 3.5),
                              Eigen::Vector3d(2.0, 0.0, 3.0)}) {
        multirotor_reference_trajectory::core::WaypointConstraint constraint;
        constraint.position = point;
        problem.constraints.push_back(constraint);
    }
    problem.segment_times = {2.0, 2.0};
    problem.start_velocity = Eigen::Vector3d(0.2, 0.0, 0.0);
    problem.start_acceleration = Eigen::Vector3d::Zero();
    problem.end_velocity = Eigen::Vector3d(0.0, -0.2, 0.0);
    problem.end_acceleration = Eigen::Vector3d::Zero();
    problem.max_iterations = 80;
    problem.time_weight = 0.1;

    multirotor_reference_trajectory::core::PiecewisePolynomialEvaluator evaluator;
    uint32_t flags = 0U;
    ASSERT_TRUE(multirotor_reference_trajectory::core::MincoWaypointSolver().solve(
        problem, evaluator, &flags));
    EXPECT_EQ(evaluator.order(), 5U);
    EXPECT_EQ(evaluator.segments().size(), 2U);

    multirotor_reference_trajectory::core::FlatOutput start;
    multirotor_reference_trajectory::core::FlatOutput middle;
    multirotor_reference_trajectory::core::FlatOutput end;
    ASSERT_TRUE(evaluator.evaluate(0.0, start));
    ASSERT_TRUE(evaluator.evaluate(evaluator.segments().front().duration, middle));
    ASSERT_TRUE(evaluator.evaluate(evaluator.duration(), end));
    EXPECT_TRUE(start.position.isApprox(problem.constraints.front().position, 1e-8));
    EXPECT_TRUE(middle.position.isApprox(problem.constraints[1].position, 1e-6));
    EXPECT_TRUE(end.position.isApprox(problem.constraints.back().position, 1e-8));
    EXPECT_TRUE(start.velocity.isApprox(problem.start_velocity, 1e-6));
    EXPECT_LT((start.acceleration - problem.start_acceleration).norm(), 1e-6);
    EXPECT_TRUE(end.velocity.isApprox(problem.end_velocity, 1e-6));
    EXPECT_LT((end.acceleration - problem.end_acceleration).norm(), 1e-6);
    EXPECT_TRUE(start.snap.allFinite());
    EXPECT_TRUE(end.snap.allFinite());
}

TEST(ReferenceTrajectoryCore, WaypointSolverMovesRegionalInteriorPointAndOptimizesTime) {
    multirotor_reference_trajectory::core::WaypointProblem problem;
    multirotor_reference_trajectory::core::WaypointConstraint start;
    start.position = Eigen::Vector3d(0.0, 0.0, 3.0);
    multirotor_reference_trajectory::core::WaypointConstraint middle;
    middle.type = multirotor_reference_trajectory::core::WaypointConstraintType::kSphere;
    middle.position = Eigen::Vector3d(1.0, 0.8, 3.3);
    middle.size = Eigen::Vector3d(0.6, 0.0, 0.0);
    multirotor_reference_trajectory::core::WaypointConstraint end;
    end.position = Eigen::Vector3d(2.0, 0.0, 3.0);
    problem.constraints = {start, middle, end};
    problem.segment_times = {1.0, 1.0};
    problem.time_weight = 0.05;
    problem.max_iterations = 80;
    problem.limits.max_velocity = 6.0;
    problem.limits.max_acceleration = 12.0;

    multirotor_reference_trajectory::core::PiecewisePolynomialEvaluator evaluator;
    uint32_t flags = 0U;
    ASSERT_TRUE(multirotor_reference_trajectory::core::MincoWaypointSolver().solve(
        problem, evaluator, &flags));
    ASSERT_EQ(evaluator.segments().size(), 2U);
    EXPECT_NEAR((evaluator.segments().front().duration + evaluator.segments().back().duration),
                evaluator.duration(), 1e-9);
    EXPECT_GT(std::abs(evaluator.duration() - 2.0), 1e-3);
    multirotor_reference_trajectory::core::FlatOutput middle_sample;
    ASSERT_TRUE(evaluator.evaluate(evaluator.segments().front().duration, middle_sample));
    EXPECT_LE((middle_sample.position - middle.position).norm(), middle.size.x() + 1e-6);
}

TEST(ReferenceTrajectoryCore, FlatnessMapperRejectsLowThrustSingularity) {
    multirotor_reference_trajectory::core::FlatOutput flat;
    flat.acceleration = -9.8066 * Eigen::Vector3d::UnitZ();

    const auto mapped =
        multirotor_reference_trajectory::core::FlatnessMapper(9.8066, 0.1).map(flat);
    EXPECT_NE(mapped.flags & multirotor_reference_trajectory::core::kFlagLowThrust, 0U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
