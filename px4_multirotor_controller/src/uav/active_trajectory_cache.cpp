#include "px4_multirotor_controller/uav/active_trajectory_cache.h"

#include <algorithm>
#include <cmath>

#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"

namespace px4_multirotor_controller {
namespace {

Eigen::Vector3d toVector(const geometry_msgs::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d toVector(const geometry_msgs::Vector3& vector) {
    return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::isfinite(siny_cosp) && std::isfinite(cosy_cosp) ? std::atan2(siny_cosp, cosy_cosp)
                                                                : 0.0;
}

multirotor_reference_trajectory::core::AnalyticType analyticType(uint16_t value) {
    switch (value) {
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HOLD:
            return multirotor_reference_trajectory::core::AnalyticType::kHold;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE:
            return multirotor_reference_trajectory::core::AnalyticType::kCircle;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_HEIGHT_CIRCLE:
            return multirotor_reference_trajectory::core::AnalyticType::kHeightCircle;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_CIRCLE_ENTRY:
            return multirotor_reference_trajectory::core::AnalyticType::kCircleEntry;
        case multirotor_reference_trajectory::AnalyticReference::ANALYTIC_FIGURE_EIGHT:
            return multirotor_reference_trajectory::core::AnalyticType::kFigureEight;
        default:
            return multirotor_reference_trajectory::core::AnalyticType::kCircleEntry;
    }
}

bool fatalReferenceFlags(uint32_t flags) {
    constexpr uint32_t kFatal = multirotor_reference_trajectory::core::kFlagInvalidInput |
                                multirotor_reference_trajectory::core::kFlagNonFinite |
                                multirotor_reference_trajectory::core::kFlagLowThrust |
                                multirotor_reference_trajectory::core::kFlagYawSingularity;
    return (flags & kFatal) != 0U;
}

void appendCoefficients(const std::vector<double>& flat, size_t offset, size_t count,
                        std::vector<double>& out) {
    out.assign(flat.begin() + static_cast<std::ptrdiff_t>(offset),
               flat.begin() + static_cast<std::ptrdiff_t>(offset + count));
}

}  // namespace

bool ActiveTrajectoryCache::updateAnalytic(
    const multirotor_reference_trajectory::AnalyticReference& msg, const ros::Time& received_time) {
    auto evaluator = std::make_unique<multirotor_reference_trajectory::core::AnalyticEvaluator>();
    uint32_t flags = 0U;
    if (!buildAnalyticEvaluator(msg, *evaluator, flags) || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    type_ = multirotor_reference_trajectory::core::TrajectoryModelType::kAnalytic;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags;
    return true;
}

bool ActiveTrajectoryCache::updatePolynomial(
    const multirotor_reference_trajectory::ActivePolynomialReference& msg,
    const ros::Time& received_time) {
    auto evaluator =
        std::make_unique<multirotor_reference_trajectory::core::PiecewisePolynomialEvaluator>();
    uint32_t flags = 0U;
    if (!buildPolynomialEvaluator(msg, *evaluator, flags) || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    type_ = multirotor_reference_trajectory::core::TrajectoryModelType::kPolynomial;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags | msg.flags;
    return true;
}

bool ActiveTrajectoryCache::updateSampled(
    const multirotor_reference_trajectory::SampledReference& msg, const ros::Time& received_time) {
    auto evaluator = std::make_unique<multirotor_reference_trajectory::core::SampledEvaluator>();
    uint32_t flags = 0U;
    if (!buildSampledEvaluator(msg, *evaluator, flags) || fatalReferenceFlags(flags)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_ = std::move(evaluator);
    type_ = multirotor_reference_trajectory::core::TrajectoryModelType::kSampled;
    trajectory_id_ = msg.trajectory_id;
    revision_ = msg.revision;
    ++sequence_;
    start_time_ = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (start_time_.isZero()) {
        start_time_ = received_time;
    }
    received_time_ = received_time;
    flags_ = flags | msg.flags;
    return true;
}

void ActiveTrajectoryCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    evaluator_.reset();
    type_ = multirotor_reference_trajectory::core::TrajectoryModelType::kNone;
    trajectory_id_ = 0U;
    revision_ = 0U;
    sequence_ = 0U;
    start_time_ = ros::Time();
    received_time_ = ros::Time();
    flags_ = 0U;
}

bool ActiveTrajectoryCache::sample(const ros::Time& now, double timeout,
                                   UavReferencePoint& sample) const {
    std::unique_ptr<multirotor_reference_trajectory::core::TrajectoryEvaluator> evaluator;
    ros::Time local_start;
    ros::Time local_received;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!evaluator_ || start_time_.isZero() ||
            (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
            return false;
        }
        evaluator = multirotor_reference_trajectory::core::cloneEvaluator(*evaluator_);
        local_start = start_time_;
        local_received = received_time_;
    }
    (void)local_received;
    if (!evaluator) {
        return false;
    }
    multirotor_reference_trajectory::core::FlatOutput flat;
    const double t = std::max(0.0, (now - local_start).toSec());
    if (!evaluator->evaluate(t, flat)) {
        return false;
    }
    sample = toPoint(flat, t);
    return true;
}

bool ActiveTrajectoryCache::sampleHorizon(
    const ros::Time& now, double stage_dt, int horizon_steps, double timeout, double gravity,
    std::vector<multirotor_reference_trajectory::UavReferenceSample>& references) const {
    if (horizon_steps <= 0 || stage_dt <= 0.0) {
        return false;
    }

    std::unique_ptr<multirotor_reference_trajectory::core::TrajectoryEvaluator> evaluator;
    ros::Time local_start;
    ros::Time local_received;
    uint32_t local_flags = 0U;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!evaluator_ || start_time_.isZero() ||
            (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
            return false;
        }
        evaluator = multirotor_reference_trajectory::core::cloneEvaluator(*evaluator_);
        local_start = start_time_;
        local_received = received_time_;
        local_flags = flags_;
    }
    (void)local_received;
    if (!evaluator || fatalReferenceFlags(local_flags)) {
        return false;
    }

    const int sample_count = horizon_steps + 2;
    const double t0 = std::max(0.0, (now - local_start).toSec());
    const double t_last = t0 + static_cast<double>(sample_count - 1) * stage_dt;
    if (evaluator->duration() > 0.0 && t_last > evaluator->duration() + 1.0e-6) {
        return false;
    }

    multirotor_reference_trajectory::core::FlatnessMapper mapper(gravity, 0.1);
    references.clear();
    references.reserve(static_cast<size_t>(sample_count));
    for (int i = 0; i < sample_count; ++i) {
        const double t = t0 + static_cast<double>(i) * stage_dt;
        multirotor_reference_trajectory::core::FlatOutput flat;
        if (!evaluator->evaluate(t, flat)) {
            return false;
        }
        const auto full = mapper.map(flat);
        if (fatalReferenceFlags(full.flags)) {
            return false;
        }
        references.push_back(toNmpcSample(full));
    }
    return true;
}

uint64_t ActiveTrajectoryCache::sequence() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sequence_;
}

uint32_t ActiveTrajectoryCache::trajectoryId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_id_;
}

uint32_t ActiveTrajectoryCache::revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return revision_;
}

bool ActiveTrajectoryCache::valid(const ros::Time& now, double timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return evaluator_ != nullptr && !start_time_.isZero() && !fatalReferenceFlags(flags_) &&
           (timeout <= 0.0 || (now - received_time_).toSec() <= timeout);
}

bool ActiveTrajectoryCache::finiteTimeRemaining(const ros::Time& now, double timeout,
                                                double& remaining) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!evaluator_ || start_time_.isZero() || fatalReferenceFlags(flags_) ||
        (timeout > 0.0 && (now - received_time_).toSec() > timeout)) {
        return false;
    }

    const double duration = evaluator_->duration();
    if (!std::isfinite(duration) || duration <= 0.0) {
        return false;
    }

    const double elapsed = std::max(0.0, (now - start_time_).toSec());
    remaining = duration - elapsed;
    return std::isfinite(remaining);
}

bool ActiveTrajectoryCache::finiteVector(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

bool ActiveTrajectoryCache::buildAnalyticEvaluator(
    const multirotor_reference_trajectory::AnalyticReference& msg,
    multirotor_reference_trajectory::core::AnalyticEvaluator& evaluator, uint32_t& flags) {
    flags = msg.flags;
    multirotor_reference_trajectory::core::AnalyticParameters params;
    params.type = analyticType(msg.analytic_type);
    params.flags = msg.flags;
    params.duration = msg.duration > 0.0 ? msg.duration : 60.0;
    params.origin = toVector(msg.origin.position);
    params.origin_yaw = yawFromQuaternion(msg.origin.orientation);
    if (msg.params.size() > 0U)
        params.radius = msg.params[0];
    if (msg.params.size() > 1U)
        params.line_speed = msg.params[1];
    if (msg.params.size() > 2U)
        params.height = msg.params[2];
    if (msg.params.size() > 3U)
        params.z_amplitude = msg.params[3];
    if (msg.params.size() > 4U)
        params.z_frequency = msg.params[4];
    if (msg.params.size() > 5U)
        params.entry_duration = msg.params[5];
    if (msg.params.size() > 6U)
        params.center.x() = msg.params[6];
    if (msg.params.size() > 7U)
        params.center.y() = msg.params[7];
    evaluator = multirotor_reference_trajectory::core::AnalyticEvaluator(params);
    flags |= multirotor_reference_trajectory::core::TrajectoryValidator::validate(
        evaluator, multirotor_reference_trajectory::core::TrajectoryLimits{}, 0.02);
    return !fatalReferenceFlags(flags);
}

bool ActiveTrajectoryCache::buildPolynomialEvaluator(
    const multirotor_reference_trajectory::ActivePolynomialReference& msg,
    multirotor_reference_trajectory::core::PiecewisePolynomialEvaluator& evaluator,
    uint32_t& flags) {
    flags = msg.flags;
    const size_t coeffs_per_segment = static_cast<size_t>(msg.order) + 1U;
    const size_t segment_count = msg.segment_durations.size();
    if (msg.order < 1U || segment_count == 0U ||
        msg.coeff_x.size() != segment_count * coeffs_per_segment ||
        msg.coeff_y.size() != msg.coeff_x.size() || msg.coeff_z.size() != msg.coeff_x.size()) {
        flags |= multirotor_reference_trajectory::core::kFlagInvalidInput;
        return false;
    }
    const bool has_yaw = msg.coeff_yaw.size() == segment_count * coeffs_per_segment;
    std::vector<multirotor_reference_trajectory::core::PolynomialSegment> segments;
    segments.reserve(segment_count);
    for (size_t i = 0; i < segment_count; ++i) {
        multirotor_reference_trajectory::core::PolynomialSegment segment;
        segment.duration = msg.segment_durations[i];
        const size_t offset = i * coeffs_per_segment;
        appendCoefficients(msg.coeff_x, offset, coeffs_per_segment, segment.x);
        appendCoefficients(msg.coeff_y, offset, coeffs_per_segment, segment.y);
        appendCoefficients(msg.coeff_z, offset, coeffs_per_segment, segment.z);
        if (has_yaw) {
            appendCoefficients(msg.coeff_yaw, offset, coeffs_per_segment, segment.yaw);
        }
        segments.push_back(std::move(segment));
    }
    if (!evaluator.setSegments(std::move(segments), msg.order)) {
        flags |= multirotor_reference_trajectory::core::kFlagInvalidInput;
        return false;
    }
    flags |= multirotor_reference_trajectory::core::TrajectoryValidator::validate(
        evaluator, multirotor_reference_trajectory::core::TrajectoryLimits{}, 0.02);
    return !fatalReferenceFlags(flags);
}

bool ActiveTrajectoryCache::buildSampledEvaluator(
    const multirotor_reference_trajectory::SampledReference& msg,
    multirotor_reference_trajectory::core::SampledEvaluator& evaluator, uint32_t& flags) {
    flags = msg.flags;
    std::vector<multirotor_reference_trajectory::core::SampledPoint> samples;
    samples.reserve(msg.points.size());
    for (const auto& point : msg.points) {
        multirotor_reference_trajectory::core::SampledPoint sample;
        sample.t = point.t_from_start;
        sample.flat.position = toVector(point.position);
        sample.flat.velocity = toVector(point.velocity);
        sample.flat.acceleration = toVector(point.acceleration);
        sample.flat.jerk = toVector(point.jerk);
        sample.flat.snap = toVector(point.snap);
        sample.flat.yaw = point.yaw;
        sample.flat.yaw_rate = point.yaw_rate;
        sample.flat.yaw_accel = point.yaw_accel;
        samples.push_back(sample);
    }
    if (!evaluator.setSamples(std::move(samples))) {
        flags |= multirotor_reference_trajectory::core::kFlagInvalidInput;
        return false;
    }
    flags |= multirotor_reference_trajectory::core::TrajectoryValidator::validate(
        evaluator, multirotor_reference_trajectory::core::TrajectoryLimits{}, 0.02);
    return !fatalReferenceFlags(flags);
}

UavReferencePoint ActiveTrajectoryCache::toPoint(
    const multirotor_reference_trajectory::core::FlatOutput& flat, double t) {
    UavReferencePoint point;
    point.t_from_start = t;
    point.position = flat.position;
    point.velocity = flat.velocity;
    point.acceleration = flat.acceleration;
    point.jerk = flat.jerk;
    point.snap = flat.snap;
    point.yaw = flat.yaw;
    point.yaw_rate = flat.yaw_rate;
    point.yaw_accel = flat.yaw_accel;
    point.flags = flat.flags;
    return point;
}

multirotor_reference_trajectory::UavReferenceSample ActiveTrajectoryCache::toNmpcSample(
    const multirotor_reference_trajectory::core::FullStateReference& full) {
    multirotor_reference_trajectory::UavReferenceSample sample;
    sample.x.setZero();
    sample.x.segment<3>(0) = full.position;
    sample.x.segment<3>(3) = full.velocity;
    sample.x.segment<4>(6) = multirotor_reference_trajectory::quatToVecWxyz(full.attitude);
    sample.x.segment<3>(10) = full.body_rate;
    sample.u << full.specific_thrust, full.angular_acceleration;
    return sample;
}

}  // namespace px4_multirotor_controller
