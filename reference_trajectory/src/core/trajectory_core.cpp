#include "reference_trajectory/core/trajectory_core.h"

#include <algorithm>
#include <cmath>

namespace reference_trajectory::core {
namespace {

constexpr double kMinDuration = 1.0e-6;
constexpr double kMinNorm = 1.0e-9;

bool finiteScalar(double value) {
    return std::isfinite(value);
}

bool finiteVector(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

double unwrapAngleNear(double value, double reference) {
    return reference + wrapAngle(value - reference);
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

void evalSeptic(const std::vector<double>& coeffs, double t, double& p, double& v, double& a,
                double& j, double& s) {
    p = polyValue(coeffs, t, 0);
    v = polyValue(coeffs, t, 1);
    a = polyValue(coeffs, t, 2);
    j = polyValue(coeffs, t, 3);
    s = polyValue(coeffs, t, 4);
}

void fillYawFromVelocity(FlatOutput& output) {
    const double vx = output.velocity.x();
    const double vy = output.velocity.y();
    const double speed_sq = vx * vx + vy * vy;
    if (speed_sq < 1.0e-8) {
        output.yaw = 0.0;
        output.yaw_rate = 0.0;
        output.yaw_accel = 0.0;
        return;
    }

    const double ax = output.acceleration.x();
    const double ay = output.acceleration.y();
    const double jx = output.jerk.x();
    const double jy = output.jerk.y();
    const double numerator = vx * ay - vy * ax;
    const double numerator_dot = vx * jy - vy * jx;
    const double denominator_dot = 2.0 * (vx * ax + vy * ay);
    output.yaw = std::atan2(vy, vx);
    output.yaw_rate = numerator / speed_sq;
    output.yaw_accel =
        (numerator_dot * speed_sq - numerator * denominator_dot) / (speed_sq * speed_sq);
}

Eigen::Vector3d unitDerivative(const Eigen::Vector3d& value, const Eigen::Vector3d& value_dot,
                               const Eigen::Vector3d& unit_value) {
    const double norm = value.norm();
    if (!finite(norm) || norm < kMinNorm) {
        return Eigen::Vector3d::Zero();
    }
    const Eigen::Matrix3d projector =
        Eigen::Matrix3d::Identity() - unit_value * unit_value.transpose();
    return projector * value_dot / norm;
}

Eigen::Vector3d unitSecondDerivative(const Eigen::Vector3d& value, const Eigen::Vector3d& value_dot,
                                     const Eigen::Vector3d& value_ddot,
                                     const Eigen::Vector3d& unit_value,
                                     const Eigen::Vector3d& unit_dot) {
    const double norm = value.norm();
    if (!finite(norm) || norm < kMinNorm) {
        return Eigen::Vector3d::Zero();
    }
    const Eigen::Matrix3d projector =
        Eigen::Matrix3d::Identity() - unit_value * unit_value.transpose();
    const Eigen::Matrix3d projector_dot =
        -(unit_dot * unit_value.transpose() + unit_value * unit_dot.transpose());
    const double norm_dot = unit_value.dot(value_dot);
    return projector_dot * value_dot / norm + projector * value_ddot / norm -
           projector * value_dot * norm_dot / (norm * norm);
}

Eigen::Vector3d normalizeOr(const Eigen::Vector3d& value, const Eigen::Vector3d& fallback) {
    const double norm = value.norm();
    if (!finite(norm) || norm < kMinNorm) {
        return fallback;
    }
    return value / norm;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& skew) {
    return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}

bool segmentAt(const std::vector<PolynomialSegment>& segments, double t, size_t& index,
               double& local_t) {
    if (segments.empty() || !finite(t)) {
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

uint32_t limitFlags(const FlatOutput& output, const TrajectoryLimits& limits) {
    uint32_t flags = kFlagNone;
    if (limits.max_velocity > 0.0 && output.velocity.norm() > limits.max_velocity) {
        flags |= kFlagVelocityLimit;
    }
    if (limits.max_acceleration > 0.0 && output.acceleration.norm() > limits.max_acceleration) {
        flags |= kFlagAccelerationLimit;
    }
    if (limits.max_jerk > 0.0 && output.jerk.norm() > limits.max_jerk) {
        flags |= kFlagJerkLimit;
    }
    if (limits.max_snap > 0.0 && output.snap.norm() > limits.max_snap) {
        flags |= kFlagSnapLimit;
    }
    return flags;
}

}  // namespace

AnalyticEvaluator::AnalyticEvaluator(AnalyticParameters params) : params_(std::move(params)) {
    params_.radius = safeRadius(params_.radius);
    params_.line_speed = std::max(0.0, params_.line_speed);
    params_.entry_duration = std::max(0.0, params_.entry_duration);
    if (!finite(params_.z_frequency) || params_.z_frequency <= 0.0) {
        params_.z_frequency = 0.5 * angularRate(params_);
    }
}

bool AnalyticEvaluator::evaluate(double t, FlatOutput& output) const {
    if (!finite(t)) {
        output.flags |= kFlagInvalidInput;
        return false;
    }
    t = clamp(t, 0.0, params_.duration);
    output = FlatOutput{};

    const double r = params_.radius;
    const double w = angularRate(params_);
    const double z_w = params_.z_frequency;
    const Eigen::Vector2d center = params_.center;
    const auto fill_circle = [&](double local_t, bool height_axis) {
        const double wt = w * local_t;
        const double zt = z_w * local_t;
        output.position.x() = center.x() + r * std::cos(wt);
        output.position.y() = center.y() + r * std::sin(wt);
        output.position.z() =
            params_.height + (height_axis ? params_.z_amplitude * std::sin(zt) : 0.0);

        output.velocity.x() = -r * w * std::sin(wt);
        output.velocity.y() = r * w * std::cos(wt);
        output.velocity.z() = height_axis ? params_.z_amplitude * z_w * std::cos(zt) : 0.0;

        output.acceleration.x() = -r * w * w * std::cos(wt);
        output.acceleration.y() = -r * w * w * std::sin(wt);
        output.acceleration.z() =
            height_axis ? -params_.z_amplitude * z_w * z_w * std::sin(zt) : 0.0;

        output.jerk.x() = r * w * w * w * std::sin(wt);
        output.jerk.y() = -r * w * w * w * std::cos(wt);
        output.jerk.z() = height_axis ? -params_.z_amplitude * z_w * z_w * z_w * std::cos(zt) : 0.0;

        output.snap.x() = r * w * w * w * w * std::cos(wt);
        output.snap.y() = r * w * w * w * w * std::sin(wt);
        output.snap.z() =
            height_axis ? params_.z_amplitude * z_w * z_w * z_w * z_w * std::sin(zt) : 0.0;
        fillYawFromVelocity(output);
    };

    switch (params_.type) {
        case AnalyticType::kHold:
            output.position = params_.origin;
            output.position.z() = params_.height;
            output.yaw = params_.origin_yaw;
            break;
        case AnalyticType::kCircle:
            fill_circle(t, false);
            break;
        case AnalyticType::kHeightCircle:
            fill_circle(t, true);
            break;
        case AnalyticType::kCircleEntry: {
            const double entry = std::max(kMinDuration, params_.entry_duration);
            if (t >= entry) {
                fill_circle(t - entry, true);
                break;
            }

            FlatOutput end;
            fill_circle(0.0, true);
            end = output;
            const Eigen::Vector3d start(params_.origin.x(), params_.origin.y(), params_.height);
            const auto cx =
                septicBoundary(start.x(), 0.0, 0.0, 0.0, end.position.x(), end.velocity.x(),
                               end.acceleration.x(), end.jerk.x(), entry);
            const auto cy =
                septicBoundary(start.y(), 0.0, 0.0, 0.0, end.position.y(), end.velocity.y(),
                               end.acceleration.y(), end.jerk.y(), entry);
            const auto cz =
                septicBoundary(start.z(), 0.0, 0.0, 0.0, end.position.z(), end.velocity.z(),
                               end.acceleration.z(), end.jerk.z(), entry);
            evalSeptic(cx, t, output.position.x(), output.velocity.x(), output.acceleration.x(),
                       output.jerk.x(), output.snap.x());
            evalSeptic(cy, t, output.position.y(), output.velocity.y(), output.acceleration.y(),
                       output.jerk.y(), output.snap.y());
            evalSeptic(cz, t, output.position.z(), output.velocity.z(), output.acceleration.z(),
                       output.jerk.z(), output.snap.z());
            fillYawFromVelocity(output);
            if (output.velocity.head<2>().squaredNorm() < 1.0e-8) {
                output.yaw = params_.origin_yaw;
            }
            break;
        }
        case AnalyticType::kFigureEight: {
            const double wt = w * t;
            output.position.x() = params_.origin.x() + r * std::sin(wt);
            output.position.y() = params_.origin.y() + 0.5 * r * std::sin(2.0 * wt);
            output.position.z() = params_.height;
            output.velocity.x() = r * w * std::cos(wt);
            output.velocity.y() = r * w * std::cos(2.0 * wt);
            output.acceleration.x() = -r * w * w * std::sin(wt);
            output.acceleration.y() = -2.0 * r * w * w * std::sin(2.0 * wt);
            output.jerk.x() = -r * w * w * w * std::cos(wt);
            output.jerk.y() = -4.0 * r * w * w * w * std::cos(2.0 * wt);
            output.snap.x() = r * w * w * w * w * std::sin(wt);
            output.snap.y() = 8.0 * r * w * w * w * w * std::sin(2.0 * wt);
            fillYawFromVelocity(output);
            break;
        }
    }

    output.flags |= params_.flags;
    if (!TrajectoryValidator::finite(output)) {
        output.flags |= kFlagNonFinite;
        return false;
    }
    return true;
}

bool PiecewisePolynomialEvaluator::setSegments(std::vector<PolynomialSegment> segments,
                                               uint8_t order) {
    segments_ = std::move(segments);
    order_ = order;
    total_duration_ = 0.0;
    flags_ = kFlagNone;
    if (segments_.empty() || order_ < 1U) {
        flags_ |= kFlagInvalidInput;
        return false;
    }
    for (const auto& segment : segments_) {
        if (!finite(segment.duration) || segment.duration <= 0.0 ||
            segment.x.size() != static_cast<size_t>(order_) + 1U ||
            segment.y.size() != segment.x.size() || segment.z.size() != segment.x.size()) {
            flags_ |= kFlagInvalidInput;
            return false;
        }
        total_duration_ += segment.duration;
    }
    return true;
}

bool PiecewisePolynomialEvaluator::evaluate(double t, FlatOutput& output) const {
    output = FlatOutput{};
    size_t index = 0U;
    double local_t = 0.0;
    if (!segmentAt(segments_, t, index, local_t)) {
        output.flags |= kFlagTimeDomain;
        return false;
    }
    const auto& segment = segments_[index];
    for (int derivative = 0; derivative <= 4; ++derivative) {
        Eigen::Vector3d value(polyValue(segment.x, local_t, derivative),
                              polyValue(segment.y, local_t, derivative),
                              polyValue(segment.z, local_t, derivative));
        switch (derivative) {
            case 0:
                output.position = value;
                break;
            case 1:
                output.velocity = value;
                break;
            case 2:
                output.acceleration = value;
                break;
            case 3:
                output.jerk = value;
                break;
            case 4:
                output.snap = value;
                break;
        }
    }
    if (!segment.yaw.empty()) {
        output.yaw = polyValue(segment.yaw, local_t, 0);
        output.yaw_rate = polyValue(segment.yaw, local_t, 1);
        output.yaw_accel = polyValue(segment.yaw, local_t, 2);
    } else {
        fillYawFromVelocity(output);
    }
    output.flags |= flags_;
    if (!TrajectoryValidator::finite(output)) {
        output.flags |= kFlagNonFinite;
        return false;
    }
    return true;
}

bool MincoWaypointSolver::solve(const WaypointProblem& problem,
                                PiecewisePolynomialEvaluator& evaluator, uint32_t* flags) const {
    uint32_t local_flags = kFlagNone;
    if (problem.waypoints.size() < 2U ||
        problem.segment_times.size() + 1U != problem.waypoints.size()) {
        local_flags |= kFlagInvalidInput;
        if (flags) {
            *flags = local_flags;
        }
        return false;
    }

    const size_t waypoint_count = problem.waypoints.size();
    std::vector<Eigen::Vector3d> velocity(waypoint_count, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> acceleration(waypoint_count, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> jerk(waypoint_count, Eigen::Vector3d::Zero());
    velocity.front() = problem.start_velocity;
    acceleration.front() = problem.start_acceleration;
    velocity.back() = problem.end_velocity;
    acceleration.back() = problem.end_acceleration;
    for (size_t i = 1; i + 1U < waypoint_count; ++i) {
        const double dt = problem.segment_times[i - 1U] + problem.segment_times[i];
        if (!finite(dt) || dt <= kMinDuration) {
            local_flags |= kFlagInvalidInput;
            if (flags) {
                *flags = local_flags;
            }
            return false;
        }
        velocity[i] = (problem.waypoints[i + 1U] - problem.waypoints[i - 1U]) / dt;
    }

    std::vector<PolynomialSegment> segments;
    segments.reserve(problem.segment_times.size());
    for (size_t i = 0; i < problem.segment_times.size(); ++i) {
        const double duration = problem.segment_times[i];
        if (!finite(duration) || duration <= kMinDuration || !finiteVector(problem.waypoints[i]) ||
            !finiteVector(problem.waypoints[i + 1U])) {
            local_flags |= kFlagInvalidInput;
            if (flags) {
                *flags = local_flags;
            }
            return false;
        }
        PolynomialSegment segment;
        segment.duration = duration;
        segment.x = septicBoundary(problem.waypoints[i].x(), velocity[i].x(), acceleration[i].x(),
                                   jerk[i].x(), problem.waypoints[i + 1U].x(), velocity[i + 1U].x(),
                                   acceleration[i + 1U].x(), jerk[i + 1U].x(), duration);
        segment.y = septicBoundary(problem.waypoints[i].y(), velocity[i].y(), acceleration[i].y(),
                                   jerk[i].y(), problem.waypoints[i + 1U].y(), velocity[i + 1U].y(),
                                   acceleration[i + 1U].y(), jerk[i + 1U].y(), duration);
        segment.z = septicBoundary(problem.waypoints[i].z(), velocity[i].z(), acceleration[i].z(),
                                   jerk[i].z(), problem.waypoints[i + 1U].z(), velocity[i + 1U].z(),
                                   acceleration[i + 1U].z(), jerk[i + 1U].z(), duration);
        segments.push_back(std::move(segment));
    }

    if (!evaluator.setSegments(std::move(segments), 7U)) {
        local_flags |= kFlagInvalidInput;
    }
    local_flags |= TrajectoryValidator::validate(evaluator, problem.limits, 0.02);
    if (flags) {
        *flags = local_flags;
    }
    return (local_flags & (kFlagInvalidInput | kFlagNonFinite)) == 0U;
}

bool SampledEvaluator::setSamples(std::vector<SampledPoint> samples) {
    samples_ = std::move(samples);
    flags_ = kFlagNone;
    duration_ = 0.0;
    if (samples_.empty()) {
        flags_ |= kFlagInvalidInput;
        return false;
    }
    double last_t = -1.0;
    for (const auto& sample : samples_) {
        if (!finite(sample.t) || sample.t < last_t || !TrajectoryValidator::finite(sample.flat)) {
            flags_ |= kFlagInvalidInput;
            return false;
        }
        last_t = sample.t;
    }
    duration_ = samples_.back().t;
    return true;
}

bool SampledEvaluator::evaluate(double t, FlatOutput& output) const {
    output = FlatOutput{};
    if (samples_.empty() || !finite(t)) {
        output.flags |= kFlagTimeDomain;
        return false;
    }
    if (samples_.size() == 1U || t <= samples_.front().t) {
        output = samples_.front().flat;
        return true;
    }
    if (t >= samples_.back().t) {
        output = samples_.back().flat;
        return true;
    }

    const auto upper =
        std::upper_bound(samples_.begin(), samples_.end(), t,
                         [](double lhs, const SampledPoint& rhs) { return lhs < rhs.t; });
    const auto prev = upper - 1;
    const auto next = upper;
    const double dt = std::max(kMinDuration, next->t - prev->t);
    const double alpha = (t - prev->t) / dt;
    output.position = (1.0 - alpha) * prev->flat.position + alpha * next->flat.position;
    output.velocity = (1.0 - alpha) * prev->flat.velocity + alpha * next->flat.velocity;
    output.acceleration = (1.0 - alpha) * prev->flat.acceleration + alpha * next->flat.acceleration;
    output.jerk = (1.0 - alpha) * prev->flat.jerk + alpha * next->flat.jerk;
    output.snap = (1.0 - alpha) * prev->flat.snap + alpha * next->flat.snap;
    output.yaw = prev->flat.yaw + alpha * wrapAngle(next->flat.yaw - prev->flat.yaw);
    output.yaw_rate = (1.0 - alpha) * prev->flat.yaw_rate + alpha * next->flat.yaw_rate;
    output.yaw_accel = (1.0 - alpha) * prev->flat.yaw_accel + alpha * next->flat.yaw_accel;
    output.flags = prev->flat.flags | next->flat.flags | flags_;
    if (!TrajectoryValidator::finite(output)) {
        output.flags |= kFlagNonFinite;
        return false;
    }
    return true;
}

uint32_t TrajectoryValidator::validate(const TrajectoryEvaluator& evaluator,
                                       const TrajectoryLimits& limits, double sample_dt) {
    uint32_t flags = evaluator.flags();
    if (!finiteScalar(evaluator.duration()) || evaluator.duration() < 0.0) {
        return flags | kFlagInvalidInput;
    }
    const double dt = std::max(1.0e-3, sample_dt);
    const int count = std::max(1, static_cast<int>(std::ceil(evaluator.duration() / dt)));
    for (int i = 0; i <= count; ++i) {
        const double t = std::min(evaluator.duration(), static_cast<double>(i) * dt);
        FlatOutput output;
        if (!evaluator.evaluate(t, output) || !finite(output)) {
            flags |= kFlagNonFinite;
            continue;
        }
        flags |= output.flags;
        flags |= limitFlags(output, limits);
    }
    return flags;
}

bool TrajectoryValidator::finite(const FlatOutput& output) {
    return finiteVector(output.position) && finiteVector(output.velocity) &&
           finiteVector(output.acceleration) && finiteVector(output.jerk) &&
           finiteVector(output.snap) && finiteScalar(output.yaw) && finiteScalar(output.yaw_rate) &&
           finiteScalar(output.yaw_accel);
}

FlatnessMapper::FlatnessMapper(double gravity, double min_specific_thrust)
    : gravity_(gravity), min_specific_thrust_(min_specific_thrust) {}

FullStateReference FlatnessMapper::map(const FlatOutput& flat) const {
    FullStateReference output;
    output.position = flat.position;
    output.velocity = flat.velocity;
    output.flags = flat.flags;
    if (!TrajectoryValidator::finite(flat)) {
        output.flags |= kFlagNonFinite;
        return output;
    }

    const Eigen::Vector3d thrust = flat.acceleration + gravity_ * Eigen::Vector3d::UnitZ();
    const double thrust_norm = thrust.norm();
    output.specific_thrust = thrust_norm;
    if (!finiteScalar(thrust_norm) || thrust_norm < min_specific_thrust_) {
        output.flags |= kFlagLowThrust;
        return output;
    }

    const Eigen::Vector3d b3 = thrust / thrust_norm;
    const Eigen::Vector3d b3_dot = unitDerivative(thrust, flat.jerk, b3);
    const Eigen::Vector3d b3_ddot = unitSecondDerivative(thrust, flat.jerk, flat.snap, b3, b3_dot);

    const double yaw = flat.yaw;
    const double yaw_rate = flat.yaw_rate;
    const double yaw_accel = flat.yaw_accel;
    const Eigen::Vector3d xc(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d xc_perp(-std::sin(yaw), std::cos(yaw), 0.0);
    const Eigen::Vector3d xc_dot = yaw_rate * xc_perp;
    const Eigen::Vector3d xc_ddot = yaw_accel * xc_perp - yaw_rate * yaw_rate * xc;

    Eigen::Vector3d y_raw = b3.cross(xc);
    Eigen::Vector3d y_raw_dot = b3_dot.cross(xc) + b3.cross(xc_dot);
    Eigen::Vector3d y_raw_ddot = b3_ddot.cross(xc) + 2.0 * b3_dot.cross(xc_dot) + b3.cross(xc_ddot);
    if (!finite(y_raw.norm()) || y_raw.norm() < 1.0e-7) {
        output.flags |= kFlagYawSingularity;
        const Eigen::Vector3d fallback = std::abs(b3.dot(Eigen::Vector3d::UnitY())) > 0.95
                                             ? Eigen::Vector3d::UnitX()
                                             : Eigen::Vector3d::UnitY();
        y_raw = b3.cross(fallback);
        y_raw_dot.setZero();
        y_raw_ddot.setZero();
    }

    const Eigen::Vector3d yb = normalizeOr(y_raw, Eigen::Vector3d::UnitY());
    const Eigen::Vector3d yb_dot = unitDerivative(y_raw, y_raw_dot, yb);
    const Eigen::Vector3d yb_ddot = unitSecondDerivative(y_raw, y_raw_dot, y_raw_ddot, yb, yb_dot);
    const Eigen::Vector3d xb = normalizeOr(yb.cross(b3), Eigen::Vector3d::UnitX());
    const Eigen::Vector3d xb_dot = yb_dot.cross(b3) + yb.cross(b3_dot);
    const Eigen::Vector3d xb_ddot =
        yb_ddot.cross(b3) + 2.0 * yb_dot.cross(b3_dot) + yb.cross(b3_ddot);

    Eigen::Matrix3d rotation;
    rotation.col(0) = xb;
    rotation.col(1) = yb;
    rotation.col(2) = b3;
    Eigen::Matrix3d rotation_dot;
    rotation_dot.col(0) = xb_dot;
    rotation_dot.col(1) = yb_dot;
    rotation_dot.col(2) = b3_dot;
    Eigen::Matrix3d rotation_ddot;
    rotation_ddot.col(0) = xb_ddot;
    rotation_ddot.col(1) = yb_ddot;
    rotation_ddot.col(2) = b3_ddot;

    const Eigen::Matrix3d omega_hat = rotation.transpose() * rotation_dot;
    const Eigen::Matrix3d alpha_hat =
        rotation_dot.transpose() * rotation_dot + rotation.transpose() * rotation_ddot;
    output.attitude = Eigen::Quaterniond(rotation);
    output.attitude.normalize();
    if (output.attitude.w() < 0.0) {
        output.attitude.coeffs() *= -1.0;
    }
    output.body_rate = vee(0.5 * (omega_hat - omega_hat.transpose()));
    output.angular_acceleration = vee(0.5 * (alpha_hat - alpha_hat.transpose()));
    if (!finiteVector(output.body_rate) || !finiteVector(output.angular_acceleration) ||
        !finite(output.attitude.norm())) {
        output.flags |= kFlagNonFinite;
    }
    return output;
}

std::unique_ptr<TrajectoryEvaluator> cloneEvaluator(const TrajectoryEvaluator& evaluator) {
    switch (evaluator.type()) {
        case TrajectoryModelType::kAnalytic: {
            const auto* analytic = dynamic_cast<const AnalyticEvaluator*>(&evaluator);
            if (!analytic) {
                return nullptr;
            }
            return std::make_unique<AnalyticEvaluator>(analytic->params());
        }
        case TrajectoryModelType::kPolynomial: {
            const auto* polynomial = dynamic_cast<const PiecewisePolynomialEvaluator*>(&evaluator);
            if (!polynomial) {
                return nullptr;
            }
            auto clone = std::make_unique<PiecewisePolynomialEvaluator>();
            (void)clone->setSegments(polynomial->segments(), polynomial->order());
            return clone;
        }
        case TrajectoryModelType::kSampled: {
            const auto* sampled = dynamic_cast<const SampledEvaluator*>(&evaluator);
            if (!sampled) {
                return nullptr;
            }
            auto clone = std::make_unique<SampledEvaluator>();
            (void)clone->setSamples(sampled->samples());
            return clone;
        }
        case TrajectoryModelType::kNone:
            break;
    }
    return nullptr;
}

}  // namespace reference_trajectory::core
