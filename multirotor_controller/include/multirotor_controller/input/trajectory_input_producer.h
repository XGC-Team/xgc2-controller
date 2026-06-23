#pragma once

#include "multirotor_controller/common/types.h"
#include "multirotor_controller/uav/reference_trajectory_buffer.h"

#include <functional>
#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <mavros_msgs/PositionTarget.h>
#include <reference_trajectory/UavBsplineTrajectory.h>
#include <reference_trajectory/UavFlatTrajectory.h>
#include <ros/ros.h>
#include <state_machine/state_machine.hpp>

namespace multirotor_controller {

class TrajectoryInputProducer {
public:
  using EventSink =
      std::function<::state_machine::Status(::state_machine::Event)>;
  using TrajectorySink = std::function<void(const MpcTrajectoryState &)>;
  using ConfigProvider = std::function<ControllerConfig()>;

  TrajectoryInputProducer(ros::NodeHandle &nh, SensorData &sensor_data,
                          ReferenceTrajectoryBuffer &reference_buffer,
                          ConfigProvider config_provider,
                          EventSink event_sink, TrajectorySink trajectory_sink,
                          uint32_t queue_size);

private:
  void algSetpointCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
  void flatTrajectoryCallback(
      const reference_trajectory::UavFlatTrajectory::ConstPtr &msg);
  void bsplineTrajectoryCallback(
      const reference_trajectory::UavBsplineTrajectory::ConstPtr &msg);
  void hoverThrustCallback(
      const hover_thrust_estimator::HoverThrustEstimate::ConstPtr &msg);
  void postInputEvent(::state_machine::EventId event_id, const char *source);

  SensorData &sensor_data_;
  ReferenceTrajectoryBuffer &reference_buffer_;
  ConfigProvider config_provider_;
  EventSink event_sink_;
  TrajectorySink trajectory_sink_;
  ros::Subscriber alg_setpoint_sub_;
  ros::Subscriber flat_trajectory_sub_;
  ros::Subscriber bspline_trajectory_sub_;
  ros::Subscriber hover_thrust_sub_;
};

} // namespace multirotor_controller
