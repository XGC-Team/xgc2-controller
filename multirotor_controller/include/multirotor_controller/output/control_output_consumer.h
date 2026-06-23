#pragma once

#include "multirotor_controller/common/ros_output_runtime.h"
#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/output/output_event_consumer.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

namespace multirotor_controller {

class ControlOutputConsumer final : public OutputEventConsumer {
public:
  ControlOutputConsumer(ros::NodeHandle &nh, RosOutputExecutor &executor,
                        DroneController &controller, uint32_t queue_size);

  bool handle(const ::state_machine::Event &event) override;

private:
  mavros_msgs::PositionTarget
  makeSetpointMessage(const Setpoint &setpoint) const;
  mavros_msgs::AttitudeTarget
  makeAttitudeRateMessage(const AttitudeRateTarget &target) const;

  RosOutputExecutor &executor_;
  DroneController &controller_;
  ros::Publisher setpoint_raw_pub_;
  ros::Publisher attitude_target_pub_;
};

} // namespace multirotor_controller
