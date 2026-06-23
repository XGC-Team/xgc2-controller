#pragma once

#include <memory>
#include <state_machine/state_machine.hpp>
#include <vector>

namespace multirotor_controller {

class OutputEventConsumer {
public:
  virtual ~OutputEventConsumer() = default;
  virtual bool handle(const ::state_machine::Event &event) = 0;
};

class OutputEventDispatcher {
public:
  void addConsumer(std::unique_ptr<OutputEventConsumer> consumer);
  void dispatch(const std::vector<::state_machine::Event> &events);

private:
  std::vector<std::unique_ptr<OutputEventConsumer>> consumers_;
};

} // namespace multirotor_controller
