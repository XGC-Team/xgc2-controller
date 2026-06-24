#include "unicycle_ugv_controller/common/reference_cache.h"

#include <geometry_msgs/Quaternion.h>

#include <algorithm>
#include <cmath>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {

namespace core = unicycle_reference_trajectory::core;

core::AnalyticType analyticType(uint16_t value) {
    switch (value) {
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_HOLD:
            return core::AnalyticType::kHold;
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE:
            return core::AnalyticType::kCircle;
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_FIGURE_EIGHT:
            return core::AnalyticType::kFigureEight;
        case unicycle_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
        default:
            return core::AnalyticType::kCircleEntry;
    }
}

bool fillAnalytic(const unicycle_reference_trajectory::AnalyticReference& msg,
                  core::AnalyticEvaluator& evaluator, uint32_t& flags) {
    flags = msg.flags;
    core::AnalyticParameters params;
    params.type = analyticType(msg.analytic_type);
    params.flags = msg.flags;
    params.duration = msg.duration > 0.0 ? msg.duration : 60.0;
    params.origin = Eigen::Vector2d(msg.origin.position.x, msg.origin.position.y);
    params.origin_yaw = yawFromQuaternion(msg.origin.orientation.x, msg.origin.orientation.y,
                                          msg.origin.orientation.z, msg.origin.orientation.w);
    if (!msg.params.empty()) {
        params.radius = msg.params[0];
    }
    if (msg.params.size() > 1U) {
        params.line_speed = msg.params[1];
    }
    if (msg.params.size() > 2U) {
        params.entry_duration = msg.params[2];
    }
    if (msg.params.size() > 3U) {
        params.center.x() = msg.params[3];
    }
    if (msg.params.size() > 4U) {
        params.center.y() = msg.params[4];
    }
    evaluator = core::AnalyticEvaluator(params);
    flags |= core::TrajectoryValidator::validate(evaluator, core::TrajectoryLimits{}, 0.02);
    return (flags & (core::kFlagInvalidInput | core::kFlagNonFinite)) == 0U;
}

bool fillPolynomial(const unicycle_reference_trajectory::ActivePolynomialReference& msg,
                    core::PiecewisePolynomialEvaluator& evaluator, uint32_t& flags) {
    flags = msg.flags;
    const size_t coeff_count = static_cast<size_t>(msg.order) + 1U;
    if (coeff_count == 0U || msg.segment_durations.empty() ||
        msg.coeff_x.size() != coeff_count * msg.segment_durations.size() ||
        msg.coeff_y.size() != coeff_count * msg.segment_durations.size() ||
        (!msg.coeff_yaw.empty() &&
         msg.coeff_yaw.size() != coeff_count * msg.segment_durations.size())) {
        flags |= core::kFlagInvalidInput;
        return false;
    }
    std::vector<core::PolynomialSegment> segments;
    segments.reserve(msg.segment_durations.size());
    for (size_t i = 0; i < msg.segment_durations.size(); ++i) {
        core::PolynomialSegment segment;
        segment.duration = msg.segment_durations[i];
        const size_t offset = i * coeff_count;
        segment.x.assign(msg.coeff_x.begin() + static_cast<long>(offset),
                         msg.coeff_x.begin() + static_cast<long>(offset + coeff_count));
        segment.y.assign(msg.coeff_y.begin() + static_cast<long>(offset),
                         msg.coeff_y.begin() + static_cast<long>(offset + coeff_count));
        if (!msg.coeff_yaw.empty()) {
            segment.yaw.assign(msg.coeff_yaw.begin() + static_cast<long>(offset),
                               msg.coeff_yaw.begin() + static_cast<long>(offset + coeff_count));
        }
        segments.push_back(std::move(segment));
    }
    if (!evaluator.setSegments(std::move(segments), msg.order)) {
        flags |= core::kFlagInvalidInput;
        return false;
    }
    flags |= core::TrajectoryValidator::validate(evaluator, core::TrajectoryLimits{}, 0.02);
    return (flags & (core::kFlagInvalidInput | core::kFlagNonFinite)) == 0U;
}

bool fillSampled(const unicycle_reference_trajectory::SampledReference& msg,
                 core::SampledEvaluator& evaluator, uint32_t& flags) {
    flags = msg.flags;
    std::vector<core::SampledPoint> samples;
    samples.reserve(msg.points.size());
    for (const auto& point : msg.points) {
        core::SampledPoint sample;
        sample.t = point.t_from_start;
        sample.reference.position = Eigen::Vector2d(point.x, point.y);
        sample.reference.velocity = Eigen::Vector2d(point.vx, point.vy);
        sample.reference.acceleration = Eigen::Vector2d(point.ax, point.ay);
        sample.reference.jerk = Eigen::Vector2d(point.jx, point.jy);
        sample.reference.yaw = point.yaw;
        sample.reference.speed = point.speed;
        sample.reference.linear_acceleration = point.linear_acceleration;
        sample.reference.yaw_rate = point.yaw_rate;
        sample.reference.yaw_acceleration = point.yaw_acceleration;
        sample.reference.curvature = point.curvature;
        samples.push_back(sample);
    }
    if (!evaluator.setSamples(std::move(samples))) {
        flags |= core::kFlagInvalidInput;
        return false;
    }
    flags |= core::TrajectoryValidator::validate(evaluator, core::TrajectoryLimits{}, 0.02);
    return (flags & (core::kFlagInvalidInput | core::kFlagNonFinite)) == 0U;
}

unicycle_reference_trajectory::UnicycleReferenceSample toSample(const core::PlanarReference& ref) {
    unicycle_reference_trajectory::UnicycleReferenceSample sample;
    sample.x << ref.position.x(), ref.position.y(), ref.yaw, ref.speed;
    sample.u << ref.linear_acceleration, ref.yaw_rate;
    return sample;
}

}  // namespace

bool ReferenceCache::updateAnalytic(const unicycle_reference_trajectory::AnalyticReference& msg,
                                    const ros::Time& received_time) {
    auto evaluator = std::make_unique<core::AnalyticEvaluator>();
    uint32_t flags = 0U;
    if (!fillAnalytic(msg, *evaluator, flags)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

bool ReferenceCache::updatePolynomial(
    const unicycle_reference_trajectory::ActivePolynomialReference& msg,
    const ros::Time& received_time) {
    auto evaluator = std::make_unique<core::PiecewisePolynomialEvaluator>();
    uint32_t flags = 0U;
    if (!fillPolynomial(msg, *evaluator, flags)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

bool ReferenceCache::updateSampled(const unicycle_reference_trajectory::SampledReference& msg,
                                   const ros::Time& received_time) {
    auto evaluator = std::make_unique<core::SampledEvaluator>();
    uint32_t flags = 0U;
    if (!fillSampled(msg, *evaluator, flags)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    start_time_ = msg.start_time;
    received_time_ = received_time;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    flags_ = flags;
    return true;
}

void ReferenceCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_.reset();
    start_time_ = ros::Time{};
    received_time_ = ros::Time{};
    trajectory_id_ = 0U;
    revision_ = 0U;
    flags_ = 0U;
}

bool ReferenceCache::activeLocked(const ros::Time& now, double timeout) const {
    if (!evaluator_ || timeout <= 0.0 || (now - received_time_).toSec() > timeout) {
        return false;
    }
    constexpr uint32_t fatal = core::kFlagInvalidInput | core::kFlagNonFinite;
    return (flags_ & fatal) == 0U;
}

bool ReferenceCache::valid(const ros::Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return activeLocked(now, timeout);
}

bool ReferenceCache::sampleHorizon(
    const ros::Time& now, double stage_dt, int horizon_steps, double timeout,
    std::vector<unicycle_reference_trajectory::UnicycleReferenceSample>& refs) const {
    if (horizon_steps <= 0 || !std::isfinite(stage_dt) || stage_dt <= 0.0) {
        return false;
    }
    std::unique_ptr<core::TrajectoryEvaluator> evaluator;
    ros::Time start_time;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!activeLocked(now, timeout)) {
            return false;
        }
        evaluator = core::cloneEvaluator(*evaluator_);
        start_time = start_time_;
    }
    if (!evaluator) {
        return false;
    }
    refs.clear();
    refs.reserve(static_cast<size_t>(horizon_steps) + 1U);
    const double base_t = std::max(0.0, (now - start_time).toSec());
    for (int i = 0; i <= horizon_steps; ++i) {
        core::PlanarReference ref;
        if (!evaluator->evaluate(base_t + static_cast<double>(i) * stage_dt, ref)) {
            return false;
        }
        refs.push_back(toSample(ref));
    }
    return true;
}

}  // namespace unicycle_ugv_controller
