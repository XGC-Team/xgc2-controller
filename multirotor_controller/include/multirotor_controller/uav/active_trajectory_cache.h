#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>

#include "multirotor_controller/common/types.h"
#include "multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "reference_trajectory/ActivePolynomialReference.h"
#include "reference_trajectory/AnalyticReference.h"
#include "reference_trajectory/SampledReference.h"
#include "reference_trajectory/core/nmpc_reference.h"
#include "reference_trajectory/core/trajectory_core.h"

namespace multirotor_controller {

struct UavReferencePoint {
    double t_from_start{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    double yaw_accel{0.0};
    uint32_t flags{0U};
};

class ActiveTrajectoryCache {
   public:
    bool updateAnalytic(const reference_trajectory::AnalyticReference& msg,
                        const ros::Time& received_time);
    bool updatePolynomial(const reference_trajectory::ActivePolynomialReference& msg,
                          const ros::Time& received_time);
    bool updateSampled(const reference_trajectory::SampledReference& msg,
                       const ros::Time& received_time);
    void clear();

    bool sample(const ros::Time& now, double timeout, UavReferencePoint& sample) const;
    bool sampleHorizon(const ros::Time& now, double stage_dt, int horizon_steps, double timeout,
                       double gravity,
                       std::vector<reference_trajectory::UavReferenceSample>& references) const;

    uint64_t sequence() const;
    uint32_t trajectoryId() const;
    uint32_t revision() const;
    bool valid(const ros::Time& now, double timeout) const;

   private:
    static bool finiteVector(const Eigen::Vector3d& value);
    static bool buildAnalyticEvaluator(const reference_trajectory::AnalyticReference& msg,
                                       reference_trajectory::core::AnalyticEvaluator& evaluator,
                                       uint32_t& flags);
    static bool buildPolynomialEvaluator(
        const reference_trajectory::ActivePolynomialReference& msg,
        reference_trajectory::core::PiecewisePolynomialEvaluator& evaluator, uint32_t& flags);
    static bool buildSampledEvaluator(const reference_trajectory::SampledReference& msg,
                                      reference_trajectory::core::SampledEvaluator& evaluator,
                                      uint32_t& flags);
    static UavReferencePoint toPoint(const reference_trajectory::core::FlatOutput& flat, double t);
    static reference_trajectory::UavReferenceSample toNmpcSample(
        const reference_trajectory::core::FullStateReference& full);

    mutable std::mutex mutex_;
    std::unique_ptr<reference_trajectory::core::TrajectoryEvaluator> evaluator_;
    reference_trajectory::core::TrajectoryModelType type_{
        reference_trajectory::core::TrajectoryModelType::kNone};
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint64_t sequence_{0U};
    ros::Time start_time_;
    ros::Time received_time_;
    uint32_t flags_{0U};
};

}  // namespace multirotor_controller
