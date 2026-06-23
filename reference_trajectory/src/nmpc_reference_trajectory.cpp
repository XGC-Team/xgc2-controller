#include "reference_trajectory/nmpc_reference_trajectory.h"

#include <cmath>

namespace reference_trajectory {
namespace {

bool isHoverType(const std::string& type) {
    return type == "hover" || type == "uav_hover" || type == "quad_hover";
}

bool isCircleType(const std::string& type) {
    return type == "circle" || type == "uav_circle" || type == "quad_circle";
}

Eigen::Matrix3d rotationFromThrustAndYaw(const Eigen::Vector3d& thrust_vec, double yaw) {
    const double thrust_norm = thrust_vec.norm();
    Eigen::Vector3d b3 = Eigen::Vector3d::UnitZ();
    if (thrust_norm > 1e-9) {
        b3 = thrust_vec / thrust_norm;
    }

    const Eigen::Vector3d b1_yaw(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d b2 = b3.cross(b1_yaw);
    if (b2.norm() < 1e-9) {
        Eigen::Vector3d fallback = Eigen::Vector3d::UnitY();
        if (std::abs(b3.dot(fallback)) > 0.95) {
            fallback = Eigen::Vector3d::UnitX();
        }
        b2 = b3.cross(fallback);
    }
    b2.normalize();

    Eigen::Matrix3d rotation;
    rotation.col(2) = b3;
    rotation.col(1) = b2;
    rotation.col(0) = b2.cross(b3);
    return projectRotation(rotation);
}

}  // namespace

UavReferenceGenerator::UavReferenceGenerator(UavReferenceConfig config)
    : config_(std::move(config)) {}

void UavReferenceGenerator::setConfig(const UavReferenceConfig& config) {
    config_ = config;
}

void UavReferenceGenerator::resetHoverPosition(const Eigen::Vector3d& position, double yaw) {
    config_.hover_position = position;
    config_.yaw = yaw;
}

UavReferenceSample UavReferenceGenerator::sample(double time_sec) const {
    UavReferenceSample out;
    const PoseReference pose = poseReference(time_sec);
    const Eigen::Vector3d omega_body = bodyRateReference(time_sec);
    const Eigen::Vector3d angular_accel = angularAccelReference(time_sec);

    out.x.segment<3>(0) = pose.position;
    out.x.segment<3>(3) = pose.velocity;
    out.x.segment<4>(6) = quatToVecWxyz(Eigen::Quaterniond(pose.rotation));
    out.x.segment<3>(10) = omega_body;
    out.u << pose.specific_thrust, angular_accel.x(), angular_accel.y(), angular_accel.z();
    return out;
}

std::string UavReferenceGenerator::resolvedType() const {
    if (config_.trajectory_id != 1) {
        switch (config_.trajectory_id) {
            case 2:
                return "line";
            case 3:
                return "lemniscate";
            case 4:
                return "helix_yz";
            case 5:
                return "helix_xy";
            case 6:
                return "torus_knot";
            default:
                return "hover";
        }
    }

    if (!config_.type.empty()) {
        return config_.type;
    }

    return "hover";
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::flatOutput(double time_sec) const {
    const std::string type = resolvedType();
    if (isCircleType(type)) {
        return circleFlatOutput(time_sec);
    }
    if (type == "line" || type == "uav_line" || type == "quad_line") {
        return lineFlatOutput(time_sec);
    }
    if (type == "lemniscate" || type == "uav_lemniscate" || type == "quad_lemniscate") {
        return lemniscateFlatOutput(time_sec);
    }
    if (type == "helix_xy" || type == "uav_helix_xy" || type == "quad_helix_xy") {
        return helixXyFlatOutput(time_sec);
    }
    if (type == "helix_yz" || type == "uav_helix_yz" || type == "quad_helix_yz") {
        return helixYzFlatOutput(time_sec);
    }
    if (type == "torus_knot" || type == "torus" || type == "uav_torus_knot" ||
        type == "quad_torus_knot") {
        return torusKnotFlatOutput(time_sec);
    }

    FlatOutput out;
    out.position = config_.hover_position;
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::circleFlatOutput(double time_sec) const {
    FlatOutput out;
    const double r = config_.radius;
    const double w = config_.omega;
    const double wt = w * time_sec;
    const double s = std::sin(wt);
    const double c = std::cos(wt);

    out.position << r * c, r * s, config_.height;
    out.velocity << -r * w * s, r * w * c, 0.0;
    out.acceleration << -r * w * w * c, -r * w * w * s, 0.0;
    out.jerk << r * w * w * w * s, -r * w * w * w * c, 0.0;
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::lineFlatOutput(double time_sec) const {
    FlatOutput out;
    const auto& line = config_.line;
    const double T = std::max(line.duration, 1e-6);
    const double t = clamp(time_sec, 0.0, T);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    const Eigen::Vector3d A = line.target_position - line.start_position - line.start_velocity * T;
    const Eigen::Vector3d D = line.start_velocity - line.target_velocity;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const Eigen::Vector3d a3 = (10.0 * A + 4.0 * D * T) / T3;
    const Eigen::Vector3d a4 = -(15.0 * A + 7.0 * D * T) / T4;
    const Eigen::Vector3d a5 = (3.0 * (2.0 * A + D * T)) / T5;

    out.position = line.start_position + line.start_velocity * t + a3 * t3 + a4 * t4 + a5 * t5;
    out.velocity = line.start_velocity + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4;
    out.acceleration = 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3;
    out.jerk = 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * t2;

    if (time_sec > T) {
        out.position = line.target_position;
        out.velocity = line.target_velocity;
        out.acceleration.setZero();
        out.jerk.setZero();
    }
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::lemniscateFlatOutput(
    double time_sec) const {
    FlatOutput out;
    const double r = config_.lemniscate.radius;
    const double w = config_.lemniscate.omega;
    const double wt = w * time_sec;
    const double s = std::sin(wt);
    const double c = std::cos(wt);

    out.position << r * s, r * s * c, config_.lemniscate.height;
    out.velocity << r * w * c, r * w * (c * c - s * s), 0.0;
    out.acceleration << -r * w * w * s, -4.0 * r * w * w * s * c, 0.0;
    out.jerk << -r * w * w * w * c, -4.0 * r * w * w * w * (c * c - s * s), 0.0;
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::helixXyFlatOutput(double time_sec) const {
    FlatOutput out;
    const double r = config_.helix_xy.radius;
    const double w = config_.helix_xy.omega;
    const double denom = std::max(config_.helix_xy.helix_scl, 1e-6);
    const double wt = w * time_sec;
    const double s = std::sin(wt);
    const double c = std::cos(wt);

    out.position << r * c, r * s, time_sec / denom;
    out.velocity << -r * w * s, r * w * c, 1.0 / denom;
    out.acceleration << -r * w * w * c, -r * w * w * s, 0.0;
    out.jerk << r * w * w * w * s, -r * w * w * w * c, 0.0;
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::helixYzFlatOutput(double time_sec) const {
    FlatOutput out;
    const double r = config_.helix_yz.radius;
    const double w = config_.helix_yz.omega;
    const double denom = std::max(config_.helix_yz.helix_scl, 1e-6);
    const double wt = w * time_sec;
    const double s = std::sin(wt);
    const double c = std::cos(wt);

    out.position << time_sec / denom, r * c, r * s;
    out.velocity << 1.0 / denom, -r * w * s, r * w * c;
    out.acceleration << 0.0, -r * w * w * c, -r * w * w * s;
    out.jerk << 0.0, r * w * w * w * s, -r * w * w * w * c;
    return out;
}

UavReferenceGenerator::FlatOutput UavReferenceGenerator::torusKnotFlatOutput(
    double time_sec) const {
    FlatOutput out;
    const double w = config_.torus_knot.omega;
    const double scale = config_.torus_knot.scale;
    const double z_offset = config_.torus_knot.z_offset;
    const double s1 = std::sin(w * time_sec);
    const double c1 = std::cos(w * time_sec);
    const double s2 = std::sin(2.0 * w * time_sec);
    const double c2 = std::cos(2.0 * w * time_sec);
    const double s3 = std::sin(3.0 * w * time_sec);
    const double c3 = std::cos(3.0 * w * time_sec);

    out.position = scale * Eigen::Vector3d(s1 + 2.0 * s2, c1 - 2.0 * c2, z_offset + s3);
    out.velocity =
        scale * Eigen::Vector3d(w * c1 + 4.0 * w * c2, -w * s1 + 4.0 * w * s2, 3.0 * w * c3);
    out.acceleration = scale * Eigen::Vector3d(-w * w * s1 - 8.0 * w * w * s2,
                                               -w * w * c1 + 8.0 * w * w * c2, -9.0 * w * w * s3);
    out.jerk =
        scale * Eigen::Vector3d(-w * w * w * c1 - 16.0 * w * w * w * c2,
                                w * w * w * s1 - 16.0 * w * w * w * s2, -27.0 * w * w * w * c3);
    return out;
}

UavReferenceGenerator::PoseReference UavReferenceGenerator::poseReference(double time_sec) const {
    if (!isHoverType(resolvedType())) {
        return flatPoseReference(time_sec);
    }
    return hoverPoseReference();
}

UavReferenceGenerator::PoseReference UavReferenceGenerator::hoverPoseReference() const {
    PoseReference out;
    out.position = config_.hover_position;
    out.velocity.setZero();
    out.rotation = yawToQuaternion(config_.yaw).toRotationMatrix();
    out.specific_thrust = config_.gravity;
    return out;
}

UavReferenceGenerator::PoseReference UavReferenceGenerator::flatPoseReference(
    double time_sec) const {
    PoseReference out;
    const FlatOutput flat = flatOutput(time_sec);
    out.position = flat.position;
    out.velocity = flat.velocity;
    const Eigen::Vector3d thrust_vec =
        flat.acceleration + config_.gravity * Eigen::Vector3d::UnitZ();
    out.specific_thrust = thrust_vec.norm();
    double yaw = config_.yaw;
    if (isCircleType(resolvedType()) && out.velocity.head<2>().norm() > 1e-6) {
        yaw = std::atan2(out.velocity.y(), out.velocity.x());
    }
    out.rotation = rotationFromThrustAndYaw(thrust_vec, yaw);
    return out;
}

Eigen::Vector3d UavReferenceGenerator::bodyRateReference(double time_sec) const {
    if (isHoverType(resolvedType())) {
        return Eigen::Vector3d::Zero();
    }
    constexpr double h = 1e-4;
    const Eigen::Matrix3d R0 = poseReference(time_sec).rotation;
    const Eigen::Matrix3d Rp = poseReference(time_sec + h).rotation;
    const Eigen::Matrix3d Rm = poseReference(time_sec - h).rotation;
    const Eigen::Matrix3d Rdot = (Rp - Rm) / (2.0 * h);
    Eigen::Matrix3d omega_x = R0.transpose() * Rdot;
    omega_x = 0.5 * (omega_x - omega_x.transpose());
    return vee(omega_x);
}

Eigen::Vector3d UavReferenceGenerator::angularAccelReference(double time_sec) const {
    if (isHoverType(resolvedType())) {
        return Eigen::Vector3d::Zero();
    }
    constexpr double h = 1e-3;
    return (bodyRateReference(time_sec + h) - bodyRateReference(time_sec - h)) / (2.0 * h);
}

UgvReferenceGenerator::UgvReferenceGenerator(UgvReferenceConfig config)
    : config_(std::move(config)) {}

UgvReferenceSample UgvReferenceGenerator::sample(double time_sec) const {
    UgvReferenceSample out;
    if (config_.type != "circle" && config_.type != "ugv_circle") {
        out.x << 0.0, 0.0, 0.0, 0.0;
        out.u << 0.0, 0.0;
        return out;
    }

    const double r = config_.radius;
    const double w = config_.omega;
    const double speed = config_.speed;
    out.x << r * std::cos(w * time_sec), r * std::sin(w * time_sec), speed,
        M_PI / 2.0 + w * time_sec;
    out.u << 0.0, w;
    return out;
}

}  // namespace reference_trajectory
