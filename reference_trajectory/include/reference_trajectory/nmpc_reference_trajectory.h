#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace reference_trajectory {

using Vector13d = Eigen::Matrix<double, 13, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

inline Eigen::Quaterniond yawToQuaternion(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d& skew) {
    return Eigen::Vector3d(skew(2, 1), skew(0, 2), skew(1, 0));
}

inline Eigen::Matrix<double, 9, 1> matToVecColumnMajor(const Eigen::Matrix3d& mat) {
    Eigen::Matrix<double, 9, 1> vec;
    vec = Eigen::Map<const Eigen::Matrix<double, 9, 1>>(mat.data());
    return vec;
}

inline Eigen::Matrix<double, 4, 1> quatToVecWxyz(const Eigen::Quaterniond& quat_in) {
    Eigen::Quaterniond quat = quat_in;
    if (!std::isfinite(quat.norm()) || quat.norm() < 1e-9) {
        quat = Eigen::Quaterniond::Identity();
    }
    quat.normalize();
    if (quat.w() < 0.0) {
        quat.coeffs() *= -1.0;
    }
    Eigen::Matrix<double, 4, 1> vec;
    vec << quat.w(), quat.x(), quat.y(), quat.z();
    return vec;
}

inline Eigen::Matrix3d projectRotation(const Eigen::Matrix3d& value) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(value, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d projected = svd.matrixU() * svd.matrixV().transpose();
    if (projected.determinant() < 0.0) {
        Eigen::Matrix3d u = svd.matrixU();
        u.col(2) *= -1.0;
        projected = u * svd.matrixV().transpose();
    }
    return projected;
}

struct UavReferenceSample {
    Vector13d x{Vector13d::Zero()};
    Vector4d u{Vector4d::Zero()};
};

struct UavReferenceConfig {
    struct LineParams {
        Eigen::Vector3d start_position{Eigen::Vector3d(0.0, 0.0, 1.0)};
        Eigen::Vector3d start_velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d target_position{Eigen::Vector3d(2.0, 0.0, 1.0)};
        Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
        double duration{8.0};
    };

    struct LemniscateParams {
        double radius{2.0};
        double omega{0.9};
        double height{1.0};
    };

    struct HelixParams {
        double radius{1.0};
        double omega{0.9};
        double helix_scl{10.0};
    };

    struct TorusKnotParams {
        double omega{0.3};
        double scale{0.6};
        double z_offset{4.0};
    };

    std::string type{"hover"};
    int trajectory_id{1};
    double time_step{0.01};
    double radius{1.2};
    double omega{0.45};
    double height{1.0};
    double yaw{0.0};
    double gravity{9.8066};
    Eigen::Vector3d hover_position{Eigen::Vector3d(0.0, 0.0, 1.0)};
    LineParams line{};
    LemniscateParams lemniscate{};
    HelixParams helix_xy{};
    HelixParams helix_yz{1.0, 1.5, 10.0};
    TorusKnotParams torus_knot{};
};

class UavReferenceGenerator {
   public:
    explicit UavReferenceGenerator(UavReferenceConfig config = {});

    void setConfig(const UavReferenceConfig& config);
    const UavReferenceConfig& config() const {
        return config_;
    }
    void resetHoverPosition(const Eigen::Vector3d& position, double yaw);
    UavReferenceSample sample(double time_sec) const;

   private:
    struct FlatOutput {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
        Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    };

    struct PoseReference {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
        double specific_thrust{0.0};
    };

    std::string resolvedType() const;
    FlatOutput flatOutput(double time_sec) const;
    FlatOutput circleFlatOutput(double time_sec) const;
    FlatOutput lineFlatOutput(double time_sec) const;
    FlatOutput lemniscateFlatOutput(double time_sec) const;
    FlatOutput helixXyFlatOutput(double time_sec) const;
    FlatOutput helixYzFlatOutput(double time_sec) const;
    FlatOutput torusKnotFlatOutput(double time_sec) const;
    PoseReference poseReference(double time_sec) const;
    PoseReference flatPoseReference(double time_sec) const;
    PoseReference hoverPoseReference() const;
    Eigen::Vector3d bodyRateReference(double time_sec) const;
    Eigen::Vector3d angularAccelReference(double time_sec) const;
    UavReferenceConfig config_;
};

struct UgvReferenceSample {
    Eigen::Matrix<double, 4, 1> x{Eigen::Matrix<double, 4, 1>::Zero()};
    Vector2d u{Vector2d::Zero()};
};

struct UgvReferenceConfig {
    std::string type{"circle"};
    double radius{2.0};
    double omega{0.25};
    double speed{0.5};
};

class UgvReferenceGenerator {
   public:
    explicit UgvReferenceGenerator(UgvReferenceConfig config = {});

    void setConfig(const UgvReferenceConfig& config) {
        config_ = config;
    }
    const UgvReferenceConfig& config() const {
        return config_;
    }
    UgvReferenceSample sample(double time_sec) const;

   private:
    UgvReferenceConfig config_;
};

}  // namespace reference_trajectory
