#pragma once

#include "multirotor_controller/common/ros_output_runtime.h"
#include "multirotor_controller/output/output_event_consumer.h"

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

namespace multirotor_controller {

class Px4ServiceOutputConsumer final : public OutputEventConsumer {
public:
  Px4ServiceOutputConsumer(ros::NodeHandle &nh, RosOutputExecutor &executor);

  bool handle(const ::state_machine::Event &event) override;
  void initializeClientsIfNeeded();

private:
  ros::NodeHandle &nh_;
  RosOutputExecutor &executor_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  bool clients_initialized_{false};
};

} // namespace multirotor_controller
