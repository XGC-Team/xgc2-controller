#include "multirotor_controller/output/reference_activation_output_consumer.h"
#include "multirotor_controller/common/types.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace multirotor_controller {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>>
makePublishTask(std::string name, ros::Publisher pub, Message msg) {
  return std::make_unique<
      ::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
      std::move(name), [pub = std::move(pub), msg = std::move(msg)](
                           ros::NodeHandle &) mutable { pub.publish(msg); });
}

double finiteOr(double value, double fallback) {
  return std::isfinite(value) ? value : fallback;
}

} // namespace

ReferenceActivationOutputConsumer::ReferenceActivationOutputConsumer(
    ros::NodeHandle &nh,
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> &executor,
    DroneController &controller, uint32_t queue_size)
    : executor_(executor), controller_(controller) {
  activation_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      "alg/reference_trajectory/activate", queue_size);
}

bool ReferenceActivationOutputConsumer::handle(
    const ::state_machine::Event &event) {
  if (event.id != output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) {
    return false;
  }

  const auto msg = makeActivationMessage(event, controller_.getSensorData());
  ROS_INFO("[ReferenceActivationOutputConsumer] Activating UAV reference at "
           "t=%.3f p=[%.3f %.3f %.3f]",
           msg.header.stamp.toSec(), msg.pose.position.x, msg.pose.position.y,
           msg.pose.position.z);
  executor_.pushTask(makePublishTask("PublishReferenceTrajectoryActivation",
                                     activation_pub_, msg));
  return true;
}

geometry_msgs::PoseStamped
ReferenceActivationOutputConsumer::makeActivationMessage(
    const ::state_machine::Event &event, const SensorData &sensor) {
  geometry_msgs::PoseStamped msg;
  const double stamp =
      event.timestamp > 0.0 ? event.timestamp : ros::Time::now().toSec();
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = "map";

  msg.pose.position.x = finiteOr(sensor.x, 0.0);
  msg.pose.position.y = finiteOr(sensor.y, 0.0);
  msg.pose.position.z = finiteOr(sensor.z, 0.0);
  msg.pose.orientation.x = finiteOr(sensor.qx, 0.0);
  msg.pose.orientation.y = finiteOr(sensor.qy, 0.0);
  msg.pose.orientation.z = finiteOr(sensor.qz, 0.0);
  msg.pose.orientation.w = finiteOr(sensor.qw, 1.0);

  const double q_norm =
      std::sqrt(msg.pose.orientation.x * msg.pose.orientation.x +
                msg.pose.orientation.y * msg.pose.orientation.y +
                msg.pose.orientation.z * msg.pose.orientation.z +
                msg.pose.orientation.w * msg.pose.orientation.w);
  if (!std::isfinite(q_norm) || q_norm < 1e-9) {
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
  } else {
    msg.pose.orientation.x /= q_norm;
    msg.pose.orientation.y /= q_norm;
    msg.pose.orientation.z /= q_norm;
    msg.pose.orientation.w /= q_norm;
  }

  return msg;
}

} // namespace multirotor_controller
