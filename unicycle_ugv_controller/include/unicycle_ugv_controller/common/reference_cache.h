#pragma once

#include <ros/time.h>

#include <memory>
#include <mutex>
#include <vector>

#include "unicycle_reference_trajectory/ActivePolynomialReference.h"
#include "unicycle_reference_trajectory/AnalyticReference.h"
#include "unicycle_reference_trajectory/SampledReference.h"
#include "unicycle_reference_trajectory/core/nmpc_reference.h"
#include "unicycle_reference_trajectory/core/trajectory_core.h"

namespace unicycle_ugv_controller {

class ReferenceCache {
   public:
    bool updateAnalytic(const unicycle_reference_trajectory::AnalyticReference& msg,
                        const ros::Time& received_time);
    bool updatePolynomial(const unicycle_reference_trajectory::ActivePolynomialReference& msg,
                          const ros::Time& received_time);
    bool updateSampled(const unicycle_reference_trajectory::SampledReference& msg,
                       const ros::Time& received_time);
    void clear();
    bool valid(const ros::Time& now, double timeout) const;
    bool sampleHorizon(
        const ros::Time& now, double stage_dt, int horizon_steps, double timeout,
        std::vector<unicycle_reference_trajectory::UnicycleReferenceSample>& refs) const;

   private:
    bool activeLocked(const ros::Time& now, double timeout) const;

    mutable std::mutex mutex_;
    std::unique_ptr<unicycle_reference_trajectory::core::TrajectoryEvaluator> evaluator_;
    ros::Time start_time_;
    ros::Time received_time_;
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint32_t flags_{0U};
};

}  // namespace unicycle_ugv_controller
