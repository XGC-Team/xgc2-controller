#pragma once

#include <ros/ros.h>
#include <state_machine/runtime/async_task_executor.hpp>

namespace multirotor_controller {

using RosOutputTask = ::state_machine::runtime::Task<ros::NodeHandle>;
using RosOutputExecutor =
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>;
using RosLambdaOutputTask =
    ::state_machine::runtime::LambdaTask<ros::NodeHandle>;

} // namespace multirotor_controller
