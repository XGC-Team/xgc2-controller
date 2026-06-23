#pragma once

#include "multirotor_controller/common/types.h"
#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/SetRuntimeParameters.h"

#include <ros/ros.h>
#include <string>
#include <vector>

namespace multirotor_controller {

class RuntimeParameterService {
public:
  RuntimeParameterService(ros::NodeHandle &nh, DroneController &controller);

private:
  bool handle(SetRuntimeParameters::Request &request,
              SetRuntimeParameters::Response &response);

  bool applyParameter(ControllerConfig &config, const std::string &name,
                      const std::string &value, std::string &error) const;
  bool validate(const ControllerConfig &config, std::string &error) const;
  std::vector<std::string> currentValues(const ControllerConfig &config) const;

  ros::ServiceServer service_;
  DroneController &controller_;
};

} // namespace multirotor_controller
