#include <gtest/gtest.h>

#include "multirotor_reference_trajectory/core/trajectory_core.h"

TEST(ReferenceTrajectorySmoke, AnalyticTypesSampleFiniteValues) {
    for (const auto type : {multirotor_reference_trajectory::core::AnalyticType::kHold,
                            multirotor_reference_trajectory::core::AnalyticType::kCircle,
                            multirotor_reference_trajectory::core::AnalyticType::kHeightCircle,
                            multirotor_reference_trajectory::core::AnalyticType::kCircleEntry,
                            multirotor_reference_trajectory::core::AnalyticType::kFigureEight}) {
        multirotor_reference_trajectory::core::AnalyticParameters params;
        params.type = type;
        params.radius = 3.0;
        params.line_speed = 3.0;
        params.height = 3.0;
        params.z_amplitude = 1.0;
        params.z_frequency = 0.5;
        multirotor_reference_trajectory::core::AnalyticEvaluator evaluator(params);
        multirotor_reference_trajectory::core::FlatOutput output;
        EXPECT_TRUE(evaluator.evaluate(0.5, output));
        EXPECT_TRUE(multirotor_reference_trajectory::core::TrajectoryValidator::finite(output));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
