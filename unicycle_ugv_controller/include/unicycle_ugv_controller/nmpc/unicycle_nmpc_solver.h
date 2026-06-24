#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

#include "unicycle_reference_trajectory/core/nmpc_reference.h"

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_unicycle_nmpc.h"
}

namespace unicycle_ugv_controller {

class UnicycleNmpcSolver {
   public:
    UnicycleNmpcSolver();
    ~UnicycleNmpcSolver();

    UnicycleNmpcSolver(const UnicycleNmpcSolver&) = delete;
    UnicycleNmpcSolver& operator=(const UnicycleNmpcSolver&) = delete;

    bool initialize();
    void resetWarmStart();
    bool solve(const Eigen::Matrix<double, 4, 1>& x0,
               const std::vector<unicycle_reference_trajectory::UnicycleReferenceSample>& refs);

    Eigen::Matrix<double, 2, 1> optimalControl() const {
        return optimal_control_;
    }
    double predictedSpeed() const {
        return predicted_speed_;
    }
    int status() const {
        return solver_status_;
    }
    double solveTimeMs() const {
        return solve_time_ms_;
    }
    static constexpr int horizonSteps() {
        return UNICYCLE_NMPC_N;
    }

   private:
    bool setInitialState(const Eigen::Matrix<double, 4, 1>& x0);
    bool setReference(int stage, const unicycle_reference_trajectory::UnicycleReferenceSample& ref);
    void setGuesses(
        const Eigen::Matrix<double, 4, 1>& x0,
        const std::vector<unicycle_reference_trajectory::UnicycleReferenceSample>& refs);
    void readSolution();
    void shiftWarmStart();
    void cleanup();

    unicycle_nmpc_solver_capsule* capsule_{nullptr};
    bool initialized_{false};
    bool have_warm_start_{false};
    int solver_status_{-1};
    double solve_time_ms_{0.0};
    std::array<Eigen::Matrix<double, 4, 1>, UNICYCLE_NMPC_N + 1> x_guess_{};
    std::array<Eigen::Matrix<double, 2, 1>, UNICYCLE_NMPC_N> u_guess_{};
    std::array<Eigen::Matrix<double, 4, 1>, UNICYCLE_NMPC_N + 1> x_solution_{};
    std::array<Eigen::Matrix<double, 2, 1>, UNICYCLE_NMPC_N> u_solution_{};
    Eigen::Matrix<double, 2, 1> optimal_control_{Eigen::Matrix<double, 2, 1>::Zero()};
    double predicted_speed_{0.0};
};

}  // namespace unicycle_ugv_controller
