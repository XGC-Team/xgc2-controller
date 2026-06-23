#include "multirotor_controller/uav/state_machine/health_monitor_state.h"
#include "multirotor_controller/drone_controller.h"

#include <cmath>
#include <utility>

namespace multirotor_controller {

using namespace event_type;

HealthMonitorState::HealthMonitorState(DroneController &controller)
    : controller_(controller) {}

::state_machine::ActionResult
HealthMonitorState::onTick(::state_machine::StateContext &ctx) {
  const auto &sd = controller_.getSensorData();
  const auto &controller_config = controller_.getConfig();
  const auto &cfg = controller_config.safety;
  auto &ss = safety_state_;

  checkSensorActiveEdge(ctx, sd.local_pos_stats, ss.was_local_pos_active,
                        SAFE_TIMEOUT_LOCAL_POS);
  checkSensorActiveEdge(ctx, sd.local_velocity_stats,
                        ss.was_local_velocity_active,
                        SAFE_TIMEOUT_LOCAL_VELOCITY);
  checkSensorActiveEdge(ctx, sd.imu_stats, ss.was_imu_active, SAFE_TIMEOUT_IMU);
  checkSensorActiveEdge(ctx, sd.state_stats, ss.was_state_active,
                        SAFE_TIMEOUT_STATE);
  checkSensorActiveEdge(ctx, sd.battery_stats, ss.was_battery_active,
                        SAFE_TIMEOUT_BATTERY);
  checkSensorActiveEdge(ctx, sd.vrpn_pose_stats, ss.was_vrpn_pose_active,
                        SAFE_TIMEOUT_VRPN_POSE);
  checkSensorActiveEdge(ctx, sd.vrpn_twist_stats, ss.was_vrpn_twist_active,
                        SAFE_TIMEOUT_VRPN_TWIST);

  if (sd.local_pos_stats.is_new) {
    const bool currently_violated =
        (sd.x < cfg.fence_x_min || sd.x > cfg.fence_x_max ||
         sd.y < cfg.fence_y_min || sd.y > cfg.fence_y_max ||
         sd.z < cfg.fence_z_min || sd.z > cfg.fence_z_max);
    if (!ss.geofence_violated && currently_violated) {
      postSafetyEvent(ctx, SAFE_GEOFENCE_VIOLATION,
                      "post geofence safety event");
    }
    ss.geofence_violated = currently_violated;
  }

  if (sd.vrpn_pose_stats.is_new) {
    if (ss.has_last_position) {
      const double dx = sd.vrpn_x - ss.last_x;
      const double dy = sd.vrpn_y - ss.last_y;
      const double dz = sd.vrpn_z - ss.last_z;
      const double position_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
      const bool jump_detected = position_jump > cfg.position_jump_threshold;
      if (!ss.position_jump_detected && jump_detected) {
        postSafetyEvent(ctx, SAFE_POSITION_JUMP, "post position jump event");
      }
      ss.position_jump_detected = jump_detected;
    }

    ss.last_x = sd.vrpn_x;
    ss.last_y = sd.vrpn_y;
    ss.last_z = sd.vrpn_z;
    ss.has_last_position = true;
  }

  if (sd.local_velocity_stats.is_new) {
    const double velocity_xy = std::sqrt(sd.vx * sd.vx + sd.vy * sd.vy);
    const bool xy_exceeded = velocity_xy > cfg.max_velocity_xy;
    if (!ss.velocity_xy_exceeded && xy_exceeded) {
      postSafetyEvent(ctx, SAFE_VELOCITY_XY_EXCEEDED,
                      "post velocity xy safety event");
    }
    ss.velocity_xy_exceeded = xy_exceeded;

    const bool z_exceeded = std::abs(sd.vz) > cfg.max_velocity_z;
    if (!ss.velocity_z_exceeded && z_exceeded) {
      postSafetyEvent(ctx, SAFE_VELOCITY_Z_EXCEEDED,
                      "post velocity z safety event");
    }
    ss.velocity_z_exceeded = z_exceeded;
  }

  if (sd.imu_stats.is_new &&
      controller_config.tracking_backend != TrackingBackend::NMPC_ATTITUDE_RATE) {
    const auto &setpoint = controller_.getSetpoint();
    const double acc_xy =
        std::sqrt(setpoint.ax * setpoint.ax + setpoint.ay * setpoint.ay);
    const bool xy_saturated = acc_xy > cfg.acc_saturation_xy;
    if (!ss.control_saturated_xy && xy_saturated) {
      postSafetyEvent(ctx, SAFE_CONTROL_SATURATION_XY,
                      "post control xy saturation event");
    }
    ss.control_saturated_xy = xy_saturated;

    const bool z_saturated = std::abs(setpoint.az) > cfg.acc_saturation_z;
    if (!ss.control_saturated_z && z_saturated) {
      postSafetyEvent(ctx, SAFE_CONTROL_SATURATION_Z,
                      "post control z saturation event");
    }
    ss.control_saturated_z = z_saturated;
  } else if (controller_config.tracking_backend ==
             TrackingBackend::NMPC_ATTITUDE_RATE) {
    ss.control_saturated_xy = false;
    ss.control_saturated_z = false;
  }

  return {};
}

void HealthMonitorState::postSafetyEvent(::state_machine::StateContext &ctx,
                                         ::state_machine::EventId event_id,
                                         const char *operation) const {
  ::state_machine::Event event(
      event_id, ::state_machine::EventTimestamp{controller_.getCurrentTime()});
  event.source = "health";
  auto status = ctx.postInternalEvent(std::move(event));
  if (!status.ok()) {
    controller_.logError("[HealthMonitorState] %s failed: %s", operation,
                         status.message.c_str());
  }
}

void HealthMonitorState::checkSensorActiveEdge(
    ::state_machine::StateContext &ctx, const SensorData::TopicStats &stats,
    bool &was_active, ::state_machine::EventId event_id) const {
  const bool currently_active = stats.is_active;
  if (was_active && !currently_active) {
    postSafetyEvent(ctx, event_id, "post sensor safety event");
  }
  was_active = currently_active;
}

} // namespace multirotor_controller
