#include "unicycle_reference_trajectory/core/trajectory_core.h"

#include <algorithm>
#include <cmath>

namespace unicycle_reference_trajectory::core {
namespace {

constexpr double kMinDuration = 1.0e-6;
constexpr double kMinSpeed = 1.0e-4;

bool finiteScalar(double value) {
    return std::isfinite(value);
}

bool finiteVector(const Eigen::Vector2d& value) {
    return value.array().isFinite().all();
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

double safeRadius(double radius) {
    return std::max(1.0e-3, std::abs(radius));
}

double angularRate(const AnalyticParameters& params) {
    return params.line_speed / safeRadius(params.radius);
}

double polyValue(const std::vector<double>& coeffs, double t, int derivative) {
    if (derivative < 0 || coeffs.empty() || derivative >= static_cast<int>(coeffs.size())) {
        return 0.0;
    }
    double value = 0.0;
    for (int i = static_cast<int>(coeffs.size()) - 1; i >= derivative; --i) {
        double factor = 1.0;
        for (int k = 0; k < derivative; ++k) {
            factor *= static_cast<double>(i - k);
        }
        value = value * t + factor * coeffs[static_cast<size_t>(i)];
    }
    return value;
}

std::vector<double> septicBoundary(double p0, double v0, double a0, double j0, double p1, double v1,
                                   double a1, double j1, double duration) {
    const double T = std::max(kMinDuration, duration);
    std::vector<double> c(8, 0.0);
    c[0] = p0;
    c[1] = v0;
    c[2] = 0.5 * a0;
    c[3] = j0 / 6.0;

    Eigen::Matrix4d A;
    Eigen::Vector4d b;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double T6 = T5 * T;
    const double T7 = T6 * T;
    A << T4, T5, T6, T7, 4.0 * T3, 5.0 * T4, 6.0 * T5, 7.0 * T6, 12.0 * T2, 20.0 * T3, 30.0 * T4,
        42.0 * T5, 24.0 * T, 60.0 * T2, 120.0 * T3, 210.0 * T4;
    b << p1 - (c[0] + c[1] * T + c[2] * T2 + c[3] * T3),
        v1 - (c[1] + 2.0 * c[2] * T + 3.0 * c[3] * T2), a1 - (2.0 * c[2] + 6.0 * c[3] * T),
        j1 - (6.0 * c[3]);
    const Eigen::Vector4d tail = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < 4; ++i) {
        c[static_cast<size_t>(i) + 4U] = tail(i);
    }
    return c;
}

bool segmentAt(const std::vector<PolynomialSegment>& segments, double t, size_t& index,
               double& local_t) {
    if (segments.empty() || !finiteScalar(t)) {
        return false;
    }
    double remaining = std::max(0.0, t);
    for (size_t i = 0; i < segments.size(); ++i) {
        const double duration = std::max(0.0, segments[i].duration);
        if (remaining <= duration || i + 1U == segments.size()) {
            index = i;
            local_t = clamp(remaining, 0.0, duration);
            return true;
        }
        remaining -= duration;
    }
    return false;
}

double yawFromVelocityOr(double vx, double vy, double fallback) {
    if (vx * vx + vy * vy < kMinSpeed * kMinSpeed) {
        return fallback;
    }
    return std::atan2(vy, vx);
}

Eigen::Vector2d unitFromYaw(double yaw) {
    return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

uint32_t limitFlags(const PlanarReference& output, const TrajectoryLimits& limits) {
    uint32_t flags = kFlagNone;
    if (limits.max_velocity > 0.0 && std::abs(output.speed) > limits.max_velocity) {
        flags |= kFlagVelocityLimit;
    }
    if (limits.max_acceleration > 0.0 &&
        std::abs(output.linear_acceleration) > limits.max_acceleration) {
        flags |= kFlagAccelerationLimit;
    }
    if (limits.max_yaw_rate > 0.0 && std::abs(output.yaw_rate) > limits.max_yaw_rate) {
        flags |= kFlagYawRateLimit;
    }
    return flags;
}

}  // namespace

void completePlanarReference(PlanarReference& output) {
    const double speed = output.velocity.norm();
    if (!finiteScalar(speed) || speed < kMinSpeed) {
        output.flags |= kFlagLowSpeedSingularity;
        output.speed = 0.0;
        output.linear_acceleration = 0.0;
        return;
    }

    const double vx = output.velocity.x();
    const double vy = output.velocity.y();
    const double ax = output.acceleration.x();
    const double ay = output.acceleration.y();
    const double jx = output.jerk.x();
    const double jy = output.jerk.y();
    const double speed_sq = speed * speed;
    const double cross_va = vx * ay - vy * ax;
    const double cross_vj = vx * jy - vy * jx;
    const double speed_sq_dot = 2.0 * (vx * ax + vy * ay);

    output.yaw = yawFromVelocityOr(vx, vy, output.yaw);
    output.speed = speed;
    output.linear_acceleration = (vx * ax + vy * ay) / speed;
    output.yaw_rate = cross_va / speed_sq;
    output.yaw_acceleration =
        (cross_vj * speed_sq - cross_va * speed_sq_dot) / (speed_sq * speed_sq);
    output.curvature = output.yaw_rate / speed;
}

AnalyticEvaluator::AnalyticEvaluator(AnalyticParameters params) : params_(std::move(params)) {
    params_.radius = safeRadius(params_.radius);
    params_.line_speed = std::max(0.0, params_.line_speed);
    params_.entry_duration = std::max(0.0, params_.entry_duration);
    if (!finiteScalar(params_.duration) || params_.duration <= 0.0) {
        params_.duration = 60.0;
    }
}

bool AnalyticEvaluator::evaluate(double t, PlanarReference& output) const {
    if (!finiteScalar(t)) {
        output.flags |= kFlagInvalidInput;
        return false;
    }
    t = clamp(t, 0.0, params_.duration);
    output = PlanarReference{};
    const double r = params_.radius;
    const double w = angularRate(params_);

    const auto fill_circle = [&](double local_t) {
        const double wt = w * local_t;
        output.position.x() = params_.center.x() + r * std::cos(wt);
        output.position.y() = params_.center.y() + r * std::sin(wt);
        output.velocity.x() = -r * w * std::sin(wt);
        output.velocity.y() = r * w * std::cos(wt);
        output.acceleration.x() = -r * w * w * std::cos(wt);
        output.acceleration.y() = -r * w * w * std::sin(wt);
        output.jerk.x() = r * w * w * w * std::sin(wt);
        output.jerk.y() = -r * w * w * w * std::cos(wt);
        completePlanarReference(output);
    };

    switch (params_.type) {
        case AnalyticType::kHold:
            output.position = params_.origin;
            output.yaw = params_.origin_yaw;
            completePlanarReference(output);
            break;
        case AnalyticType::kCircle:
            fill_circle(t);
            break;
        case AnalyticType::kCircleEntry: {
            const double entry = std::max(kMinDuration, params_.entry_duration);
            if (t >= entry) {
                fill_circle(t - entry);
                break;
            }
            PlanarReference end;
            fill_circle(0.0);
            end = output;
            const auto cx =
                septicBoundary(params_.origin.x(), 0.0, 0.0, 0.0, end.position.x(),
                               end.velocity.x(), end.acceleration.x(), end.jerk.x(), entry);
            const auto cy =
                septicBoundary(params_.origin.y(), 0.0, 0.0, 0.0, end.position.y(),
                               end.velocity.y(), end.acceleration.y(), end.jerk.y(), entry);
            output = PlanarReference{};
            output.position << polyValue(cx, t, 0), polyValue(cy, t, 0);
            output.velocity << polyValue(cx, t, 1), polyValue(cy, t, 1);
            output.acceleration << polyValue(cx, t, 2), polyValue(cy, t, 2);
            output.jerk << polyValue(cx, t, 3), polyValue(cy, t, 3);
            output.yaw = params_.origin_yaw;
            completePlanarReference(output);
            break;
        }
        case AnalyticType::kFigureEight: {
            const double w8 = w;
            const double wt = w8 * t;
            output.position.x() = params_.center.x() + r * std::sin(wt);
            output.position.y() = params_.center.y() + 0.5 * r * std::sin(2.0 * wt);
            output.velocity.x() = r * w8 * std::cos(wt);
            output.velocity.y() = r * w8 * std::cos(2.0 * wt);
            output.acceleration.x() = -r * w8 * w8 * std::sin(wt);
            output.acceleration.y() = -2.0 * r * w8 * w8 * std::sin(2.0 * wt);
            output.jerk.x() = -r * w8 * w8 * w8 * std::cos(wt);
            output.jerk.y() = -4.0 * r * w8 * w8 * w8 * std::cos(2.0 * wt);
            completePlanarReference(output);
            break;
        }
    }
    output.flags |= params_.flags;
    return TrajectoryValidator::finite(output);
}

bool PiecewisePolynomialEvaluator::setSegments(std::vector<PolynomialSegment> segments,
                                               uint8_t order) {
    if (segments.empty() || order == 0U) {
        flags_ |= kFlagInvalidInput;
        return false;
    }
    total_duration_ = 0.0;
    for (const auto& segment : segments) {
        if (!finiteScalar(segment.duration) || segment.duration <= kMinDuration ||
            segment.x.size() != static_cast<size_t>(order) + 1U ||
            segment.y.size() != static_cast<size_t>(order) + 1U ||
            (!segment.yaw.empty() && segment.yaw.size() != static_cast<size_t>(order) + 1U)) {
            flags_ |= kFlagInvalidInput;
            return false;
        }
        total_duration_ += segment.duration;
    }
    order_ = order;
    segments_ = std::move(segments);
    return true;
}

bool PiecewisePolynomialEvaluator::evaluate(double t, PlanarReference& output) const {
    size_t index = 0U;
    double local_t = 0.0;
    if (!segmentAt(segments_, t, index, local_t)) {
        output.flags |= kFlagInvalidInput;
        return false;
    }
    const auto& segment = segments_[index];
    output = PlanarReference{};
    output.position << polyValue(segment.x, local_t, 0), polyValue(segment.y, local_t, 0);
    output.velocity << polyValue(segment.x, local_t, 1), polyValue(segment.y, local_t, 1);
    output.acceleration << polyValue(segment.x, local_t, 2), polyValue(segment.y, local_t, 2);
    output.jerk << polyValue(segment.x, local_t, 3), polyValue(segment.y, local_t, 3);
    if (!segment.yaw.empty()) {
        output.yaw = wrapAngle(polyValue(segment.yaw, local_t, 0));
        output.yaw_rate = polyValue(segment.yaw, local_t, 1);
        output.yaw_acceleration = polyValue(segment.yaw, local_t, 2);
        output.speed = output.velocity.norm();
        output.linear_acceleration = output.speed > kMinSpeed
                                         ? output.velocity.dot(output.acceleration) / output.speed
                                         : 0.0;
        output.curvature =
            std::abs(output.speed) > kMinSpeed ? output.yaw_rate / output.speed : 0.0;
    } else {
        completePlanarReference(output);
    }
    output.flags |= flags_;
    return TrajectoryValidator::finite(output);
}

bool MincoWaypointSolver::solve(const WaypointProblem& problem,
                                PiecewisePolynomialEvaluator& evaluator, uint32_t* flags) const {
    uint32_t local_flags = problem.flags;
    if (problem.constraints.size() < 2U) {
        local_flags |= kFlagInvalidInput;
        if (flags) {
            *flags |= local_flags;
        }
        return false;
    }

    std::vector<double> times = problem.segment_times;
    if (times.empty()) {
        times.reserve(problem.constraints.size() - 1U);
        for (size_t i = 0; i + 1U < problem.constraints.size(); ++i) {
            const double distance =
                (problem.constraints[i + 1U].position - problem.constraints[i].position).norm();
            const double speed = std::max(0.1, problem.desired_speed);
            times.push_back(std::max(problem.min_segment_time, distance / speed));
        }
    }
    if (times.size() + 1U != problem.constraints.size()) {
        local_flags |= kFlagInvalidInput;
        if (flags) {
            *flags |= local_flags;
        }
        return false;
    }

    const size_t count = problem.constraints.size();
    std::vector<Eigen::Vector2d> velocities(count, Eigen::Vector2d::Zero());
    std::vector<Eigen::Vector2d> accelerations(count, Eigen::Vector2d::Zero());
    velocities.front() = problem.start_velocity;
    velocities.back() = problem.end_velocity;
    accelerations.front() = problem.start_acceleration;
    accelerations.back() = problem.end_acceleration;
    for (size_t i = 1; i + 1U < count; ++i) {
        const double dt = std::max(kMinDuration, times[i - 1U] + times[i]);
        velocities[i] =
            (problem.constraints[i + 1U].position - problem.constraints[i - 1U].position) / dt;
    }

    std::vector<PolynomialSegment> segments;
    segments.reserve(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
        const double T = std::max(problem.min_segment_time, times[i]);
        PolynomialSegment segment;
        segment.duration = T;
        const auto& p0 = problem.constraints[i].position;
        const auto& p1 = problem.constraints[i + 1U].position;
        const auto& v0 = velocities[i];
        const auto& v1 = velocities[i + 1U];
        const auto& a0 = accelerations[i];
        const auto& a1 = accelerations[i + 1U];
        segment.x = septicBoundary(p0.x(), v0.x(), a0.x(), 0.0, p1.x(), v1.x(), a1.x(), 0.0, T);
        segment.y = septicBoundary(p0.y(), v0.y(), a0.y(), 0.0, p1.y(), v1.y(), a1.y(), 0.0, T);
        segment.yaw = septicBoundary(problem.constraints[i].yaw, 0.0, 0.0, 0.0,
                                     problem.constraints[i + 1U].yaw, 0.0, 0.0, 0.0, T);
        segments.push_back(std::move(segment));
    }
    const bool ok = evaluator.setSegments(std::move(segments), 7U);
    local_flags |= TrajectoryValidator::validate(evaluator, problem.limits, 0.02);
    if (flags) {
        *flags |= local_flags;
    }
    return ok && (local_flags & (kFlagInvalidInput | kFlagNonFinite)) == 0U;
}

bool SampledEvaluator::setSamples(std::vector<SampledPoint> samples) {
    if (samples.empty()) {
        flags_ |= kFlagInvalidInput;
        return false;
    }
    std::sort(samples.begin(), samples.end(),
              [](const SampledPoint& lhs, const SampledPoint& rhs) { return lhs.t < rhs.t; });
    double last_t = -1.0;
    for (auto& sample : samples) {
        completePlanarReference(sample.reference);
        if (!finiteScalar(sample.t) || sample.t <= last_t ||
            !TrajectoryValidator::finite(sample.reference)) {
            flags_ |= kFlagInvalidInput;
            return false;
        }
        last_t = sample.t;
    }
    duration_ = samples.back().t;
    samples_ = std::move(samples);
    return true;
}

bool SampledEvaluator::evaluate(double t, PlanarReference& output) const {
    if (samples_.empty() || !finiteScalar(t)) {
        output.flags |= kFlagInvalidInput;
        return false;
    }
    if (t <= samples_.front().t) {
        output = samples_.front().reference;
        return true;
    }
    if (t >= samples_.back().t) {
        output = samples_.back().reference;
        return true;
    }
    for (size_t i = 0; i + 1U < samples_.size(); ++i) {
        const auto& a = samples_[i];
        const auto& b = samples_[i + 1U];
        if (t < a.t || t > b.t) {
            continue;
        }
        const double ratio = (t - a.t) / std::max(kMinDuration, b.t - a.t);
        output.position = (1.0 - ratio) * a.reference.position + ratio * b.reference.position;
        output.velocity = (1.0 - ratio) * a.reference.velocity + ratio * b.reference.velocity;
        output.acceleration =
            (1.0 - ratio) * a.reference.acceleration + ratio * b.reference.acceleration;
        output.jerk = (1.0 - ratio) * a.reference.jerk + ratio * b.reference.jerk;
        output.yaw = a.reference.yaw + wrapAngle(b.reference.yaw - a.reference.yaw) * ratio;
        completePlanarReference(output);
        output.flags |= flags_;
        return true;
    }
    output.flags |= kFlagTimeDomain;
    return false;
}

uint32_t TrajectoryValidator::validate(const TrajectoryEvaluator& evaluator,
                                       const TrajectoryLimits& limits, double sample_dt) {
    uint32_t flags = evaluator.flags();
    const double duration = evaluator.duration();
    if (!finiteScalar(duration) || duration < 0.0) {
        return flags | kFlagInvalidInput;
    }
    sample_dt = std::max(1.0e-3, sample_dt);
    const int count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    for (int i = 0; i <= count; ++i) {
        const double t = duration * static_cast<double>(i) / static_cast<double>(count);
        PlanarReference output;
        if (!evaluator.evaluate(t, output) || !TrajectoryValidator::finite(output)) {
            flags |= kFlagNonFinite;
            continue;
        }
        flags |= limitFlags(output, limits);
    }
    return flags;
}

bool TrajectoryValidator::finite(const PlanarReference& output) {
    return finiteVector(output.position) && finiteVector(output.velocity) &&
           finiteVector(output.acceleration) && finiteVector(output.jerk) &&
           finiteScalar(output.yaw) && finiteScalar(output.speed) &&
           finiteScalar(output.linear_acceleration) && finiteScalar(output.yaw_rate) &&
           finiteScalar(output.yaw_acceleration) && finiteScalar(output.curvature);
}

std::unique_ptr<TrajectoryEvaluator> cloneEvaluator(const TrajectoryEvaluator& evaluator) {
    if (evaluator.type() == TrajectoryModelType::kAnalytic) {
        const auto* analytic = dynamic_cast<const AnalyticEvaluator*>(&evaluator);
        if (analytic) {
            return std::make_unique<AnalyticEvaluator>(analytic->params());
        }
    }
    if (evaluator.type() == TrajectoryModelType::kPolynomial) {
        const auto* polynomial = dynamic_cast<const PiecewisePolynomialEvaluator*>(&evaluator);
        if (polynomial) {
            auto clone = std::make_unique<PiecewisePolynomialEvaluator>();
            clone->setSegments(polynomial->segments(), polynomial->order());
            return clone;
        }
    }
    if (evaluator.type() == TrajectoryModelType::kSampled) {
        const auto* sampled = dynamic_cast<const SampledEvaluator*>(&evaluator);
        if (sampled) {
            auto clone = std::make_unique<SampledEvaluator>();
            clone->setSamples(sampled->samples());
            return clone;
        }
    }
    return nullptr;
}

}  // namespace unicycle_reference_trajectory::core
