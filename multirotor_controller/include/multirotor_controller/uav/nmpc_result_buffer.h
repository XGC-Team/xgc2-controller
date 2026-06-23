#pragma once

#include "multirotor_controller/common/types.h"

#include <mutex>
#include <ros/ros.h>

namespace multirotor_controller {

struct NmpcSolveResult {
  uint64_t sequence{0};
  bool success{false};
  bool timed_out{false};
  int solver_status{0};
  double solve_time_ms{0.0};
  ros::Time stamp;
  AttitudeRateTarget target;
};

class NmpcResultBuffer {
public:
  void store(const NmpcSolveResult &result);
  bool consumeNewerThan(uint64_t sequence, NmpcSolveResult &result) const;
  bool hasFreshSuccess(const ros::Time &now, double timeout) const;

private:
  mutable std::mutex mutex_;
  NmpcSolveResult latest_;
  bool has_result_{false};
};

} // namespace multirotor_controller
