#pragma once

#include "multirotor_controller/common/ros_output_runtime.h"
#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/input/command_input_producer.h"
#include "multirotor_controller/input/sensor_input_producer.h"
#include "multirotor_controller/input/trajectory_input_producer.h"
#include "multirotor_controller/output/output_event_consumer.h"
#include "multirotor_controller/output/px4_service_output_consumer.h"
#include "multirotor_controller/service/runtime_parameter_service.h"
#include <ros1_utils/topic_stats.h>

#include <memory>
#include <ros/ros.h>

namespace multirotor_controller {

class DroneRosNode {
public:
  explicit DroneRosNode(ros::NodeHandle &nh);
  ~DroneRosNode();

  void run(double frequency);

private:
  void controlLoopCallback();
  void dispatchOutputEvents(const std::vector<::state_machine::Event> &events);
  void loadControllerConfig();
  void loadVrpnQualityConfig();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  SensorData sensor_data_;
  DroneController controller_;

  RosOutputExecutor output_event_executor_;
  OutputEventDispatcher output_event_dispatcher_;
  Px4ServiceOutputConsumer *px4_service_consumer_{nullptr};

  ros1_utils::PositionQualityStats vrpn_quality_stats_;
  bool debug_print_{true};

  std::unique_ptr<SensorInputProducer> sensor_input_producer_;
  std::unique_ptr<CommandInputProducer> command_input_producer_;
  std::unique_ptr<TrajectoryInputProducer> trajectory_input_producer_;
  std::unique_ptr<RuntimeParameterService> runtime_parameter_service_;
};

} // namespace multirotor_controller
