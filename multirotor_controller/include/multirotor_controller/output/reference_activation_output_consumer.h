#pragma once

#include "multirotor_controller/common/ros_output_runtime.h"
#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/output/output_event_consumer.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace multirotor_controller {

class ReferenceActivationOutputConsumer final : public OutputEventConsumer {
public:
  ReferenceActivationOutputConsumer(ros::NodeHandle &nh,
                                    RosOutputExecutor &executor,
                                    DroneController &controller,
                                    uint32_t queue_size);

  bool handle(const ::state_machine::Event &event) override;

private:
  static geometry_msgs::PoseStamped
  makeActivationMessage(const ::state_machine::Event &event,
                        const SensorData &sensor);

  RosOutputExecutor &executor_;
  DroneController &controller_;
  ros::Publisher activation_pub_;
};

} // namespace multirotor_controller
