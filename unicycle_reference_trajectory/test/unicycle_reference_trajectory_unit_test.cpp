#include <gtest/gtest.h>

#include <cmath>

#include "unicycle_reference_trajectory/core/trajectory_core.h"

namespace {

TEST(UnicycleReferenceTrajectoryCore, CircleProvidesUnicycleReferences) {
    unicycle_reference_trajectory::core::AnalyticParameters params;
    params.type = unicycle_reference_trajectory::core::AnalyticType::kCircle;
    params.radius = 3.0;
    params.line_speed = 2.0;
    params.center = Eigen::Vector2d::Zero();

    unicycle_reference_trajectory::core::AnalyticEvaluator evaluator(params);
    unicycle_reference_trajectory::core::PlanarReference output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.speed, 2.0, 1e-12);
    EXPECT_NEAR(output.yaw_rate, 2.0 / 3.0, 1e-12);
    EXPECT_TRUE(std::isfinite(output.curvature));
}

TEST(UnicycleReferenceTrajectoryCore, WaypointSolverProducesPlanarPolynomial) {
    unicycle_reference_trajectory::core::WaypointProblem problem;
    for (const auto& point :
         {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(2.0, 0.0)}) {
        unicycle_reference_trajectory::core::WaypointConstraint constraint;
        constraint.position = point;
        problem.constraints.push_back(constraint);
    }
    problem.segment_times = {2.0, 2.0};
    problem.start_velocity = Eigen::Vector2d(0.2, 0.0);
    problem.end_velocity = Eigen::Vector2d(0.0, -0.2);

    unicycle_reference_trajectory::core::PiecewisePolynomialEvaluator evaluator;
    uint32_t flags = 0U;
    ASSERT_TRUE(unicycle_reference_trajectory::core::MincoWaypointSolver().solve(problem, evaluator,
                                                                                 &flags));
    EXPECT_EQ(evaluator.order(), 7U);
    EXPECT_EQ(evaluator.segments().size(), 2U);

    unicycle_reference_trajectory::core::PlanarReference start;
    unicycle_reference_trajectory::core::PlanarReference middle;
    unicycle_reference_trajectory::core::PlanarReference end;
    ASSERT_TRUE(evaluator.evaluate(0.0, start));
    ASSERT_TRUE(evaluator.evaluate(2.0, middle));
    ASSERT_TRUE(evaluator.evaluate(4.0, end));
    EXPECT_TRUE(start.position.isApprox(problem.constraints.front().position, 1e-9));
    EXPECT_TRUE(middle.position.isApprox(problem.constraints[1].position, 1e-9));
    EXPECT_TRUE(end.position.isApprox(problem.constraints.back().position, 1e-9));
    EXPECT_TRUE(start.velocity.isApprox(problem.start_velocity, 1e-9));
    EXPECT_TRUE(end.velocity.isApprox(problem.end_velocity, 1e-9));
}

TEST(UnicycleReferenceTrajectoryCore, HoldReportsLowSpeedSingularity) {
    unicycle_reference_trajectory::core::AnalyticParameters params;
    params.type = unicycle_reference_trajectory::core::AnalyticType::kHold;
    unicycle_reference_trajectory::core::AnalyticEvaluator evaluator(params);
    unicycle_reference_trajectory::core::PlanarReference output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NE(output.flags & unicycle_reference_trajectory::core::kFlagLowSpeedSingularity, 0U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
