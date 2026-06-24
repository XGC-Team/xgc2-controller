#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>

#include "multirotor_reference_trajectory/ActivePolynomialReference.h"
#include "multirotor_reference_trajectory/AnalyticReference.h"
#include "multirotor_reference_trajectory/SampledReference.h"
#include "multirotor_reference_trajectory/core/nmpc_reference.h"
#include "multirotor_reference_trajectory/core/trajectory_core.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"

namespace px4_multirotor_controller {

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
    bool updateAnalytic(const multirotor_reference_trajectory::AnalyticReference& msg,
                        const ros::Time& received_time);
    bool updatePolynomial(const multirotor_reference_trajectory::ActivePolynomialReference& msg,
                          const ros::Time& received_time);
    bool updateSampled(const multirotor_reference_trajectory::SampledReference& msg,
                       const ros::Time& received_time);
    void clear();

    bool sample(const ros::Time& now, double timeout, UavReferencePoint& sample) const;
    bool sampleHorizon(
        const ros::Time& now, double stage_dt, int horizon_steps, double timeout, double gravity,
        std::vector<multirotor_reference_trajectory::UavReferenceSample>& references) const;

    uint64_t sequence() const;
    uint32_t trajectoryId() const;
    uint32_t revision() const;
    bool valid(const ros::Time& now, double timeout) const;
    bool finiteTimeRemaining(const ros::Time& now, double timeout, double& remaining) const;

   private:
    static bool finiteVector(const Eigen::Vector3d& value);
    static bool buildAnalyticEvaluator(
        const multirotor_reference_trajectory::AnalyticReference& msg,
        multirotor_reference_trajectory::core::AnalyticEvaluator& evaluator, uint32_t& flags);
    static bool buildPolynomialEvaluator(
        const multirotor_reference_trajectory::ActivePolynomialReference& msg,
        multirotor_reference_trajectory::core::PiecewisePolynomialEvaluator& evaluator,
        uint32_t& flags);
    static bool buildSampledEvaluator(
        const multirotor_reference_trajectory::SampledReference& msg,
        multirotor_reference_trajectory::core::SampledEvaluator& evaluator, uint32_t& flags);
    static UavReferencePoint toPoint(const multirotor_reference_trajectory::core::FlatOutput& flat,
                                     double t);
    static multirotor_reference_trajectory::UavReferenceSample toNmpcSample(
        const multirotor_reference_trajectory::core::FullStateReference& full);

    mutable std::mutex mutex_;
    std::unique_ptr<multirotor_reference_trajectory::core::TrajectoryEvaluator> evaluator_;
    multirotor_reference_trajectory::core::TrajectoryModelType type_{
        multirotor_reference_trajectory::core::TrajectoryModelType::kNone};
    uint32_t trajectory_id_{0U};
    uint32_t revision_{0U};
    uint64_t sequence_{0U};
    ros::Time start_time_;
    ros::Time received_time_;
    uint32_t flags_{0U};
};

}  // namespace px4_multirotor_controller
