#pragma once

#include "multirotor_controller/common/types.h"
#include <state_machine/state_machine.hpp>

namespace multirotor_controller {

class DroneController;

class HealthMonitorState final : public ::state_machine::State {
public:
  explicit HealthMonitorState(DroneController &controller);

  std::string name() const override { return "HealthMonitor"; }

protected:
  ::state_machine::ActionResult
  onTick(::state_machine::StateContext &ctx) override;

private:
  struct SafetyState {
    bool was_local_pos_active{false};
    bool was_local_velocity_active{false};
    bool was_imu_active{false};
    bool was_state_active{false};
    bool was_battery_active{false};
    bool was_vrpn_pose_active{false};
    bool was_vrpn_twist_active{false};

    double last_x{0.0};
    double last_y{0.0};
    double last_z{0.0};
    bool has_last_position{false};

    bool geofence_violated{false};
    bool velocity_xy_exceeded{false};
    bool velocity_z_exceeded{false};
    bool control_saturated_xy{false};
    bool control_saturated_z{false};
    bool position_jump_detected{false};
  };

  void postSafetyEvent(::state_machine::StateContext &ctx,
                       ::state_machine::EventId event_id,
                       const char *operation) const;
  void checkSensorActiveEdge(::state_machine::StateContext &ctx,
                             const SensorData::TopicStats &stats,
                             bool &was_active,
                             ::state_machine::EventId event_id) const;

  DroneController &controller_;
  SafetyState safety_state_;
};

} // namespace multirotor_controller
