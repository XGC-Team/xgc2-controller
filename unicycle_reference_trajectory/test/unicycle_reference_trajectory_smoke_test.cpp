#include <gtest/gtest.h>

#include "unicycle_reference_trajectory/core/trajectory_core.h"

TEST(UnicycleReferenceTrajectorySmoke, AnalyticEvaluatorsDoNotProduceNonFiniteOutput) {
    for (const auto type : {unicycle_reference_trajectory::core::AnalyticType::kHold,
                            unicycle_reference_trajectory::core::AnalyticType::kCircle,
                            unicycle_reference_trajectory::core::AnalyticType::kCircleEntry,
                            unicycle_reference_trajectory::core::AnalyticType::kFigureEight}) {
        unicycle_reference_trajectory::core::AnalyticParameters params;
        params.type = type;
        params.radius = 3.0;
        params.line_speed = 1.5;
        params.entry_duration = 2.0;
        unicycle_reference_trajectory::core::AnalyticEvaluator evaluator(params);
        unicycle_reference_trajectory::core::PlanarReference output;
        ASSERT_TRUE(evaluator.evaluate(0.5, output));
        EXPECT_TRUE(unicycle_reference_trajectory::core::TrajectoryValidator::finite(output));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
