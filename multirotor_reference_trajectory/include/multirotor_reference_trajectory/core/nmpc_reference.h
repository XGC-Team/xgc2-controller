#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace multirotor_reference_trajectory {

using Vector13d = Eigen::Matrix<double, 13, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;

inline Eigen::Matrix<double, 4, 1> quatToVecWxyz(const Eigen::Quaterniond& quat_in) {
    Eigen::Quaterniond quat = quat_in;
    if (!std::isfinite(quat.norm()) || quat.norm() < 1.0e-9) {
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

struct UavReferenceSample {
    Vector13d x{Vector13d::Zero()};
    Vector4d u{Vector4d::Zero()};
};

}  // namespace multirotor_reference_trajectory
