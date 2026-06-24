#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <vector>

namespace multirotor_reference_trajectory::core {

enum class TrajectoryModelType : uint8_t {
    kNone = 0,
    kAnalytic = 1,
    kPolynomial = 2,
    kSampled = 3,
};

enum class AnalyticType : uint16_t {
    kHold = 0,
    kCircle = 1,
    kHeightCircle = 2,
    kCircleEntry = 3,
    kFigureEight = 4,
};

enum TrajectoryFlag : uint32_t {
    kFlagNone = 0U,
    kFlagInvalidInput = 1U << 0U,
    kFlagTimeDomain = 1U << 1U,
    kFlagNonFinite = 1U << 2U,
    kFlagLowThrust = 1U << 3U,
    kFlagYawSingularity = 1U << 4U,
    kFlagVelocityLimit = 1U << 5U,
    kFlagAccelerationLimit = 1U << 6U,
    kFlagJerkLimit = 1U << 7U,
    kFlagSnapLimit = 1U << 8U,
    kFlagBodyRateLimit = 1U << 9U,
    kFlagTiltLimit = 1U << 10U,
    kFlagThrustLimit = 1U << 11U,
    kFlagOptimizationFailure = 1U << 12U,
};

struct FlatOutput {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
    Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    double yaw_accel{0.0};
    uint32_t flags{kFlagNone};
};

struct FullStateReference {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
    double specific_thrust{0.0};
    uint32_t flags{kFlagNone};
};

struct TrajectoryLimits {
    double max_velocity{0.0};
    double max_acceleration{0.0};
    double max_jerk{0.0};
    double max_snap{0.0};
    double min_specific_thrust{0.1};
    double max_specific_thrust{0.0};
    double max_body_rate{0.0};
    double max_tilt{0.0};
};

enum class WaypointConstraintType : uint8_t {
    kPoint = 0,
    kSphere = 1,
    kBox = 2,
    kGate = 3,
};

struct WaypointConstraint {
    WaypointConstraintType type{WaypointConstraintType::kPoint};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d size{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class TrajectoryEvaluator {
   public:
    virtual ~TrajectoryEvaluator() = default;
    virtual bool evaluate(double t, FlatOutput& output) const = 0;
    virtual double duration() const = 0;
    virtual TrajectoryModelType type() const = 0;
    virtual uint32_t flags() const = 0;
};

struct AnalyticParameters {
    AnalyticType type{AnalyticType::kCircleEntry};
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double origin_yaw{0.0};
    double radius{3.0};
    double line_speed{3.0};
    double height{3.0};
    double z_amplitude{1.0};
    double z_frequency{0.5};
    double entry_duration{5.0};
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
};

class AnalyticEvaluator final : public TrajectoryEvaluator {
   public:
    explicit AnalyticEvaluator(AnalyticParameters params = {});

    bool evaluate(double t, FlatOutput& output) const override;
    double duration() const override {
        return params_.duration;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kAnalytic;
    }
    uint32_t flags() const override {
        return params_.flags;
    }
    const AnalyticParameters& params() const {
        return params_;
    }

   private:
    AnalyticParameters params_;
};

struct PolynomialSegment {
    double duration{0.0};
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> yaw;
};

class PiecewisePolynomialEvaluator final : public TrajectoryEvaluator {
   public:
    bool setSegments(std::vector<PolynomialSegment> segments, uint8_t order);
    bool evaluate(double t, FlatOutput& output) const override;
    double duration() const override {
        return total_duration_;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kPolynomial;
    }
    uint32_t flags() const override {
        return flags_;
    }
    uint8_t order() const {
        return order_;
    }
    const std::vector<PolynomialSegment>& segments() const {
        return segments_;
    }

   private:
    std::vector<PolynomialSegment> segments_;
    double total_duration_{0.0};
    uint8_t order_{0U};
    uint32_t flags_{kFlagNone};
};

struct WaypointProblem {
    std::vector<WaypointConstraint> constraints;
    std::vector<double> segment_times;
    Eigen::Vector3d start_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d end_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d end_acceleration{Eigen::Vector3d::Zero()};
    TrajectoryLimits limits{};
    double desired_speed{1.0};
    double time_weight{1.0};
    double dynamic_penalty_weight{1000.0};
    int max_iterations{80};
    int integral_resolution{12};
    double rel_cost_tol{1.0e-5};
    double min_segment_time{0.1};
    double max_segment_time{30.0};
    double validation_sample_dt{0.02};
    uint32_t flags{kFlagNone};
};

class MincoWaypointSolver final {
   public:
    bool solve(const WaypointProblem& problem, PiecewisePolynomialEvaluator& evaluator,
               uint32_t* flags = nullptr) const;
};

struct SampledPoint {
    double t{0.0};
    FlatOutput flat{};
};

class SampledEvaluator final : public TrajectoryEvaluator {
   public:
    bool setSamples(std::vector<SampledPoint> samples);
    bool evaluate(double t, FlatOutput& output) const override;
    double duration() const override {
        return duration_;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kSampled;
    }
    uint32_t flags() const override {
        return flags_;
    }
    const std::vector<SampledPoint>& samples() const {
        return samples_;
    }

   private:
    std::vector<SampledPoint> samples_;
    double duration_{0.0};
    uint32_t flags_{kFlagNone};
};

class TrajectoryValidator final {
   public:
    static uint32_t validate(const TrajectoryEvaluator& evaluator, const TrajectoryLimits& limits,
                             double sample_dt);
    static bool finite(const FlatOutput& output);
};

class FlatnessMapper final {
   public:
    explicit FlatnessMapper(double gravity = 9.8066, double min_specific_thrust = 0.1);
    FullStateReference map(const FlatOutput& flat) const;

   private:
    double gravity_;
    double min_specific_thrust_;
};

std::unique_ptr<TrajectoryEvaluator> cloneEvaluator(const TrajectoryEvaluator& evaluator);

}  // namespace multirotor_reference_trajectory::core
