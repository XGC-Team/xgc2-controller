#include <gtest/gtest.h>

#include <cmath>

#include "reference_trajectory/core/trajectory_core.h"

namespace {

TEST(ReferenceTrajectoryCore, HeightCircleProvidesHighOrderDerivatives) {
    reference_trajectory::core::AnalyticParameters params;
    params.type = reference_trajectory::core::AnalyticType::kHeightCircle;
    params.radius = 3.0;
    params.line_speed = 3.0;
    params.height = 3.0;
    params.z_amplitude = 1.0;
    params.z_frequency = 0.5;

    reference_trajectory::core::AnalyticEvaluator evaluator(params);
    reference_trajectory::core::FlatOutput output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.velocity.y(), 3.0, 1e-12);
    EXPECT_TRUE(output.snap.allFinite());
    EXPECT_TRUE(std::isfinite(output.yaw_rate));
    EXPECT_TRUE(std::isfinite(output.yaw_accel));
}

TEST(ReferenceTrajectoryCore, WaypointSolverProducesSepticPolynomial) {
    reference_trajectory::core::WaypointProblem problem;
    problem.waypoints = {Eigen::Vector3d(0.0, 0.0, 3.0), Eigen::Vector3d(1.0, 1.0, 3.5),
                         Eigen::Vector3d(2.0, 0.0, 3.0)};
    problem.segment_times = {2.0, 2.0};

    reference_trajectory::core::PiecewisePolynomialEvaluator evaluator;
    uint32_t flags = 0U;
    ASSERT_TRUE(
        reference_trajectory::core::MincoWaypointSolver().solve(problem, evaluator, &flags));
    EXPECT_EQ(evaluator.order(), 7U);
    EXPECT_EQ(evaluator.segments().size(), 2U);

    reference_trajectory::core::FlatOutput start;
    reference_trajectory::core::FlatOutput end;
    ASSERT_TRUE(evaluator.evaluate(0.0, start));
    ASSERT_TRUE(evaluator.evaluate(4.0, end));
    EXPECT_TRUE(start.position.isApprox(problem.waypoints.front(), 1e-9));
    EXPECT_TRUE(end.position.isApprox(problem.waypoints.back(), 1e-9));
    EXPECT_TRUE(start.snap.allFinite());
    EXPECT_TRUE(end.snap.allFinite());
}

TEST(ReferenceTrajectoryCore, FlatnessMapperRejectsLowThrustSingularity) {
    reference_trajectory::core::FlatOutput flat;
    flat.acceleration = -9.8066 * Eigen::Vector3d::UnitZ();

    const auto mapped = reference_trajectory::core::FlatnessMapper(9.8066, 0.1).map(flat);
    EXPECT_NE(mapped.flags & reference_trajectory::core::kFlagLowThrust, 0U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
