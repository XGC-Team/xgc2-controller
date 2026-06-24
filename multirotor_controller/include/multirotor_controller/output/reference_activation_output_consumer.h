#pragma once

#include "multirotor_controller/drone_controller.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

namespace multirotor_controller {

class ReferenceActivationOutputConsumer final
    : public ::state_machine::runtime::EventConsumer {
public:
  ReferenceActivationOutputConsumer(ros::NodeHandle &nh,
                                    ::state_machine::runtime::
                                        AsyncTaskExecutor<ros::NodeHandle>
                                            &executor,
                                    DroneController &controller,
                                    uint32_t queue_size);

  std::string name() const override {
    return "ReferenceActivationOutputConsumer";
  }
  bool handle(const ::state_machine::Event &event) override;

private:
  static geometry_msgs::PoseStamped
  makeActivationMessage(const ::state_machine::Event &event,
                        const SensorData &sensor);

  ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> &executor_;
  DroneController &controller_;
  ros::Publisher activation_pub_;
};

} // namespace multirotor_controller
