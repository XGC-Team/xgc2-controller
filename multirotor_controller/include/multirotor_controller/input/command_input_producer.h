#pragma once

#include <functional>
#include <ros/ros.h>
#include <state_machine/state_machine.hpp>
#include <std_msgs/String.h>

namespace multirotor_controller {

class CommandInputProducer {
public:
  using EventSink =
      std::function<::state_machine::Status(::state_machine::Event)>;

  CommandInputProducer(ros::NodeHandle &nh, EventSink event_sink,
                       uint32_t queue_size);

private:
  void commandCallback(const std_msgs::String::ConstPtr &msg);

  EventSink event_sink_;
  ros::Subscriber command_sub_;
};

} // namespace multirotor_controller
