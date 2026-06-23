#include <gtest/gtest.h>

#include <cmath>

#include "reference_trajectory/nmpc_reference_trajectory.h"
#include "reference_trajectory/uniform_velocity_reference_trajectory.h"

namespace {

TEST(ReferenceTrajectoryUnit, HelperFunctionsProduceValidRotations) {
    EXPECT_DOUBLE_EQ(reference_trajectory::clamp(2.0, -1.0, 1.0), 1.0);

    const Eigen::Quaterniond yaw_quaternion = reference_trajectory::yawToQuaternion(M_PI / 2.0);
    const Eigen::Vector3d x_axis = yaw_quaternion * Eigen::Vector3d::UnitX();
    EXPECT_NEAR(x_axis.x(), 0.0, 1e-12);
    EXPECT_NEAR(x_axis.y(), 1.0, 1e-12);

    Eigen::Matrix3d nearly_rotation = Eigen::Matrix3d::Identity();
    nearly_rotation(0, 1) = 0.02;
    const Eigen::Matrix3d projected = reference_trajectory::projectRotation(nearly_rotation);
    EXPECT_NEAR((projected.transpose() * projected - Eigen::Matrix3d::Identity()).norm(), 0.0,
                1e-12);
    EXPECT_NEAR(projected.determinant(), 1.0, 1e-12);
}

TEST(ReferenceTrajectoryUnit, UavHoverSampleMatchesConfiguredPose) {
    reference_trajectory::UavReferenceConfig config;
    config.type = "hover";
    config.hover_position = Eigen::Vector3d(1.0, 2.0, 3.0);
    config.yaw = M_PI / 2.0;
    config.gravity = 9.81;

    const reference_trajectory::UavReferenceSample sample =
        reference_trajectory::UavReferenceGenerator(config).sample(4.0);

    EXPECT_TRUE(sample.x.segment<3>(0).isApprox(config.hover_position, 1e-12));
    EXPECT_TRUE(sample.x.segment<3>(3).isZero(0.0));
    EXPECT_NEAR(sample.u(0), 9.81, 1e-12);
    EXPECT_TRUE(sample.u.tail<3>().isZero(0.0));
}

TEST(ReferenceTrajectoryUnit, UavCircleSampleHasExpectedFlatOutputAtStart) {
    reference_trajectory::UavReferenceConfig config;
    config.type = "circle";
    config.radius = 2.0;
    config.omega = 0.5;
    config.height = 1.5;

    const reference_trajectory::UavReferenceSample sample =
        reference_trajectory::UavReferenceGenerator(config).sample(0.0);

    EXPECT_NEAR(sample.x(0), 2.0, 1e-12);
    EXPECT_NEAR(sample.x(1), 0.0, 1e-12);
    EXPECT_NEAR(sample.x(2), 1.5, 1e-12);
    EXPECT_NEAR(sample.x(3), 0.0, 1e-12);
    EXPECT_NEAR(sample.x(4), 1.0, 1e-12);
    EXPECT_GT(sample.u(0), config.gravity);
    EXPECT_TRUE(sample.x.allFinite());
    EXPECT_TRUE(sample.u.allFinite());
}

TEST(ReferenceTrajectoryUnit, UgvCircleSampleMatchesAnalyticState) {
    reference_trajectory::UgvReferenceConfig config;
    config.type = "circle";
    config.radius = 3.0;
    config.omega = 0.2;
    config.speed = 0.7;

    const reference_trajectory::UgvReferenceSample sample =
        reference_trajectory::UgvReferenceGenerator(config).sample(0.0);

    EXPECT_NEAR(sample.x(0), 3.0, 1e-12);
    EXPECT_NEAR(sample.x(1), 0.0, 1e-12);
    EXPECT_NEAR(sample.x(2), 0.7, 1e-12);
    EXPECT_NEAR(sample.x(3), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(sample.u(0), 0.0, 1e-12);
    EXPECT_NEAR(sample.u(1), 0.2, 1e-12);
}

TEST(ReferenceTrajectoryUnit, UniformVelocityTrajectoryStepsAndPredicts) {
    reference_trajectory::UniformVelocityReferenceTrajectory trajectory(1.0, 0.5, 4);
    trajectory.initialize(Eigen::Vector3d::Zero());
    trajectory.setNewTarget(Eigen::Vector3d(2.0, 0.0, 0.0), false);

    EXPECT_FALSE(trajectory.hasReachedTarget());
    EXPECT_NEAR(trajectory.getRemainingDistance(), 2.0, 1e-12);
    EXPECT_EQ(trajectory.getPredictedTrajectory().rows(), 6);
    EXPECT_EQ(trajectory.getPredictedTrajectory().cols(), 4);
    EXPECT_NEAR(trajectory.getStateAtStep(2)(0), 1.0, 1e-12);

    trajectory.step(0.5);
    EXPECT_NEAR(trajectory.getCurrentPosition().x(), 0.5, 1e-12);
    EXPECT_NEAR(trajectory.getCurrentVelocity().x(), 1.0, 1e-12);

    trajectory.step(10.0);
    EXPECT_TRUE(trajectory.hasReachedTarget());
    EXPECT_TRUE(trajectory.getCurrentPosition().isApprox(Eigen::Vector3d(2.0, 0.0, 0.0), 1e-12));
    EXPECT_TRUE(trajectory.getCurrentVelocity().isZero(0.0));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
