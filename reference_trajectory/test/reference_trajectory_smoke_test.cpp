#include <gtest/gtest.h>

#include <vector>

#include "reference_trajectory/nmpc_reference_trajectory.h"

TEST(ReferenceTrajectorySmoke, AllUavTrajectoryIdsSampleFiniteValues) {
    reference_trajectory::UavReferenceConfig config;
    reference_trajectory::UavReferenceGenerator generator(config);

    for (int trajectory_id = 1; trajectory_id <= 6; ++trajectory_id) {
        config.trajectory_id = trajectory_id;
        generator.setConfig(config);
        const reference_trajectory::UavReferenceSample sample = generator.sample(0.5);
        EXPECT_TRUE(sample.x.allFinite()) << trajectory_id;
        EXPECT_TRUE(sample.u.allFinite()) << trajectory_id;
    }
}

TEST(ReferenceTrajectorySmoke, UnknownUgvTypeFallsBackToZeroCommand) {
    reference_trajectory::UgvReferenceConfig config;
    config.type = "unsupported";
    const reference_trajectory::UgvReferenceSample sample =
        reference_trajectory::UgvReferenceGenerator(config).sample(1.0);

    EXPECT_TRUE(sample.x.isZero(0.0));
    EXPECT_TRUE(sample.u.isZero(0.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
