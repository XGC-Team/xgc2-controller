#include <ros/ros.h>

#include "reference_trajectory/reference_trajectory_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "reference_trajectory_node");
    ros::NodeHandle nh;
    reference_trajectory::ReferenceTrajectoryNode node(nh);
    node.run();
    return 0;
}
