#pragma once

#include <ros/ros.h>

#include <string>

#include "reference_trajectory/AnalyticReference.h"
#include "reference_trajectory/SampledReference.h"
#include "reference_trajectory/WaypointReferenceRequest.h"
#include "reference_trajectory/reference_trajectory_runtime.h"
#include "std_msgs/Empty.h"

namespace reference_trajectory {

class ReferenceInputProducer {
   public:
    ReferenceInputProducer(ros::NodeHandle& nh, ReferenceTrajectoryRuntime& runtime,
                           const std::string& analytic_topic, const std::string& waypoint_topic,
                           const std::string& sampled_topic, const std::string& reset_topic,
                           uint32_t queue_size);

   private:
    void analyticCallback(const AnalyticReference::ConstPtr& msg);
    void waypointCallback(const WaypointReferenceRequest::ConstPtr& msg);
    void sampledCallback(const SampledReference::ConstPtr& msg);
    void resetCallback(const std_msgs::Empty::ConstPtr& msg);
    void post(uint32_t event_id, const char* source);

    ReferenceTrajectoryRuntime& runtime_;
    ros::Subscriber analytic_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber sampled_sub_;
    ros::Subscriber reset_sub_;
};

}  // namespace reference_trajectory
