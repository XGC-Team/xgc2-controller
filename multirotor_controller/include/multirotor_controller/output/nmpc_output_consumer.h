#pragma once

#include "multirotor_controller/drone_controller.h"
#include "multirotor_controller/output/output_event_consumer.h"
#include "multirotor_controller/uav/nmpc_tracking_backend.h"

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace multirotor_controller {

class NmpcOutputConsumer final : public OutputEventConsumer {
public:
  using EventSink =
      std::function<::state_machine::Status(::state_machine::Event)>;

  NmpcOutputConsumer(DroneController &controller, EventSink event_sink);
  ~NmpcOutputConsumer() override;

  bool handle(const ::state_machine::Event &event) override;

private:
  struct Request {
    uint64_t sequence{0};
    ros::Time now;
    SensorData sensor;
    std::vector<reference_trajectory::UavReferenceSample> references;
  };

  void workerLoop();
  void reject(uint64_t sequence, int solver_status);
  void postResultEvent(uint64_t sequence, bool success);

  DroneController &controller_;
  EventSink event_sink_;
  UavNmpcTrackingBackend backend_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  bool stop_{false};
  bool busy_{false};
  bool has_pending_{false};
  Request pending_;
};

} // namespace multirotor_controller
