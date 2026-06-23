#ifndef REFERENCE_TRAJECTORY_UNIFORM_VELOCITY_REFERENCE_TRAJECTORY_H
#define REFERENCE_TRAJECTORY_UNIFORM_VELOCITY_REFERENCE_TRAJECTORY_H

#include <ros/ros.h>

#include <Eigen/Dense>

namespace reference_trajectory {

/**
 * @brief 独立的匀速领导者参考轨迹生成器
 *
 * 设计理念：
 * - 作为领导者，完全独立运行，不依赖外部状态反馈
 * - 内部维护自己的位置状态，通过匀速积分更新
 * - 每次step()同时更新当前状态和预测时域轨迹
 * - 为跟随者提供高效的轨迹访问接口
 */
class UniformVelocityReferenceTrajectory {
   public:
    /**
     * @brief 构造函数
     * @param speed 匀速运动速度 (m/s)
     * @param dt 时间步长 (s)
     * @param horizon 预测步数
     */
    UniformVelocityReferenceTrajectory(double speed = 1.0, double dt = 0.1, int horizon = 100)
        : speed_(speed),
          dt_(dt),
          horizon_(horizon),
          current_time_(0.0),
          remaining_distance_(0.0),
          remaining_time_(0.0),
          is_initialized_(false),
          is_moving_(false),
          predicted_trajectory_(Eigen::MatrixXd::Zero(6, horizon_)) {
        current_pos_.setZero();
        current_vel_.setZero();
        target_pos_.setZero();
        target_direction_.setZero();
    }

    /**
     * @brief 初始化起始位置（仅在首次启动时调用）
     * @param initial_pos 初始位置 (3D)
     */
    void initialize(const Eigen::Vector3d& initial_pos) {
        current_pos_ = initial_pos;
        current_vel_.setZero();
        target_pos_ = initial_pos;
        target_direction_.setZero();
        current_time_ = 0.0;
        remaining_distance_ = 0.0;
        remaining_time_ = 0.0;
        is_moving_ = false;
        is_initialized_ = true;

        updatePredictedTrajectory();

        ROS_INFO("[UniformVelocityRef] Initialized at position: [%.3f, %.3f, %.3f]",
                 current_pos_.x(), current_pos_.y(), current_pos_.z());
    }

    /**
     * @brief 设置新的目标点（从当前内部状态出发）
     * @param target_pos 目标位置 (3D)
     */
    void setNewTarget(const Eigen::Vector3d& target_pos, bool emit_logs = true) {
        if (!is_initialized_) {
            if (emit_logs) {
                ROS_WARN("[UniformVelocityRef] Not initialized! Call initialize() first.");
            }
            return;
        }

        target_pos_ = target_pos;

        const Eigen::Vector3d delta = target_pos_ - current_pos_;
        remaining_distance_ = delta.norm();

        if (remaining_distance_ > 1e-2) {
            target_direction_ = delta.normalized();
            current_vel_ = target_direction_ * speed_;
            remaining_time_ = remaining_distance_ / speed_;
            is_moving_ = true;

            updatePredictedTrajectory();

            if (emit_logs) {
                ROS_INFO(
                    "[UniformVelocityRef] New target: [%.3f, %.3f, %.3f], distance: %.3fm, time: "
                    "%.3fs",
                    target_pos_.x(), target_pos_.y(), target_pos_.z(), remaining_distance_,
                    remaining_time_);
                ROS_INFO(
                    "[UniformVelocityRef] Starting from: [%.3f, %.3f, %.3f], velocity: [%.3f, "
                    "%.3f, %.3f]m/s",
                    current_pos_.x(), current_pos_.y(), current_pos_.z(), current_vel_.x(),
                    current_vel_.y(), current_vel_.z());
            }
        } else {
            target_direction_.setZero();
            current_vel_.setZero();
            remaining_time_ = 0.0;
            is_moving_ = false;
            updatePredictedTrajectory();
            if (emit_logs) {
                ROS_INFO(
                    "[UniformVelocityRef] Target too close (%.3fm), staying at current position",
                    remaining_distance_);
            }
        }
    }

    /**
     * @brief 时间步进（每个控制周期调用一次）
     * 同时更新：
     * - 当前位置和速度
     * - 预测时域内的完整轨迹
     * @param dt 时间增量 (s)，默认使用构造时的dt
     */
    void step(double dt = -1.0) {
        if (!is_initialized_) {
            return;
        }

        if (!is_moving_) {
            return;
        }

        const double time_step = (dt > 0.0) ? dt : dt_;
        const double step_distance = speed_ * time_step;

        if (step_distance >= remaining_distance_) {
            current_pos_ = target_pos_;
            current_vel_.setZero();
            remaining_distance_ = 0.0;
            remaining_time_ = 0.0;
            is_moving_ = false;

            ROS_INFO("[UniformVelocityRef] Reached target: [%.3f, %.3f, %.3f]", current_pos_.x(),
                     current_pos_.y(), current_pos_.z());
        } else {
            current_pos_ += current_vel_ * time_step;
            remaining_distance_ -= step_distance;
            remaining_time_ -= time_step;
        }

        updatePredictedTrajectory();
    }

    /**
     * @brief 将内部轨迹状态推进到指定仿真时间
     * @param simulation_time 绝对仿真时间 (s)
     */
    void advanceTo(double simulation_time) {
        if (!is_initialized_) {
            return;
        }

        const double dt = simulation_time - current_time_;
        if (dt <= 0.0) {
            return;
        }

        current_time_ = simulation_time;
        step(dt);
    }

    /**
     * @brief 获取当前时刻的状态（位置+速度）
     * @return 6维向量 [px, py, pz, vx, vy, vz]
     */
    Eigen::VectorXd getCurrentState() const {
        Eigen::VectorXd state(6);
        state << current_pos_.x(), current_pos_.y(), current_pos_.z(), current_vel_.x(),
            current_vel_.y(), current_vel_.z();
        return state;
    }

    /**
     * @brief 获取预测时域轨迹（高效访问，无需重新计算）
     * @return 6 × horizon 矩阵的常量引用，每列为一个时刻的状态 [px, py, pz, vx, vy, vz]
     */
    const Eigen::MatrixXd& getPredictedTrajectory() const {
        return predicted_trajectory_;
    }

    /**
     * @brief 获取预测时域中第i步的状态
     * @param step_index 步数索引 [0, horizon-1]
     * @return 6维状态向量
     */
    Eigen::VectorXd getStateAtStep(int step_index) const {
        if (step_index < 0 || step_index >= horizon_) {
            ROS_WARN("[UniformVelocityRef] Step index %d out of range [0, %d)", step_index,
                     horizon_);
            return getCurrentState();
        }
        return predicted_trajectory_.col(step_index);
    }

    /**
     * @brief 检查是否已到达目标点
     * @return true 表示已到达
     */
    bool hasReachedTarget() const {
        return is_initialized_ && !is_moving_;
    }

    /**
     * @brief 获取剩余距离
     * @return 剩余距离 (m)
     */
    double getRemainingDistance() const {
        return remaining_distance_;
    }

    /**
     * @brief 获取剩余时间
     * @return 剩余时间 (s)
     */
    double getRemainingTime() const {
        return remaining_time_;
    }

    /**
     * @brief 获取当前位置
     * @return 3维位置向量
     */
    Eigen::Vector3d getCurrentPosition() const {
        return current_pos_;
    }

    /**
     * @brief 获取当前速度
     * @return 3维速度向量
     */
    Eigen::Vector3d getCurrentVelocity() const {
        return current_vel_;
    }

    /**
     * @brief 获取目标位置
     * @return 3维目标位置向量
     */
    Eigen::Vector3d getTargetPosition() const {
        return target_pos_;
    }

    /**
     * @brief 获取预测时域步数
     * @return 预测步数
     */
    int getHorizon() const {
        return horizon_;
    }

    /**
     * @brief 获取时间步长
     * @return 时间步长 (s)
     */
    double getTimeStep() const {
        return dt_;
    }

   private:
    // 参数
    double speed_;         // 运动速度 (m/s)
    double dt_;            // 时间步长 (s)
    int horizon_;          // 预测步数
    double current_time_;  // 当前仿真时间 (s)

    // 内部状态（领导者自主维护）
    Eigen::Vector3d current_pos_;  // 当前位置
    Eigen::Vector3d current_vel_;  // 当前速度

    // 目标状态
    Eigen::Vector3d target_pos_;        // 目标位置
    Eigen::Vector3d target_direction_;  // 目标方向（单位向量）

    double remaining_distance_;  // 剩余距离 (m)
    double remaining_time_;      // 剩余时间 (s)

    bool is_initialized_;  // 是否已初始化
    bool is_moving_;       // 是否正在移动

    // 预测轨迹缓存（在step()中更新）
    Eigen::MatrixXd predicted_trajectory_;  // 6 × horizon，每列为[px, py, pz, vx, vy, vz]

    /**
     * @brief 更新预测时域轨迹
     * 从当前状态开始，预测未来horizon步的轨迹
     */
    void updatePredictedTrajectory() {
        for (int i = 0; i < horizon_; ++i) {
            predicted_trajectory_.col(i) = computeStateAtStep(i);
        }
    }

    /**
     * @brief 计算未来某时刻的状态
     * @param steps 未来的步数
     * @return 6维状态向量
     */
    Eigen::VectorXd computeStateAtStep(int steps) const {
        Eigen::VectorXd state(6);

        if (!is_moving_) {
            state << current_pos_.x(), current_pos_.y(), current_pos_.z(), 0.0, 0.0, 0.0;
            return state;
        }

        const double future_time = steps * dt_;
        const double future_distance = speed_ * future_time;

        if (future_distance < remaining_distance_) {
            const Eigen::Vector3d future_pos = current_pos_ + current_vel_ * future_time;
            state << future_pos.x(), future_pos.y(), future_pos.z(), current_vel_.x(),
                current_vel_.y(), current_vel_.z();
        } else {
            state << target_pos_.x(), target_pos_.y(), target_pos_.z(), 0.0, 0.0, 0.0;
        }

        return state;
    }
};

}  // namespace reference_trajectory

#endif  // REFERENCE_TRAJECTORY_UNIFORM_VELOCITY_REFERENCE_TRAJECTORY_H
