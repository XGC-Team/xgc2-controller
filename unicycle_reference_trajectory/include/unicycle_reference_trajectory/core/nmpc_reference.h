#pragma once

#include <Eigen/Dense>

namespace unicycle_reference_trajectory {

using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;

struct UnicycleReferenceSample {
    // [x, y, yaw, linear_speed]
    Vector4d x{Vector4d::Zero()};
    // [linear_acceleration, yaw_rate]
    Vector2d u{Vector2d::Zero()};
};

}  // namespace unicycle_reference_trajectory
