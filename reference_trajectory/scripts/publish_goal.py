#!/usr/bin/env python3
"""
Publish Goal to /move_base_simple/goal

向 /move_base_simple/goal 发布一个目标点

用法：
    rosrun reference_trajectory publish_goal.py
    roslaunch reference_trajectory ugv_goal.launch goal_x:=17.0 goal_y:=0.0
"""

import rospy
import re
from geometry_msgs.msg import PoseStamped


def publish_goal():
    # 从参数服务器读取目标位置
    goal_x = rospy.get_param('~goal_x', 17.0)
    goal_y = rospy.get_param('~goal_y', 0.0)
    min_subscribers = int(rospy.get_param('~min_subscribers', 1))
    required_mpc_subscribers = int(rospy.get_param('~required_mpc_subscribers', 0))
    mpc_subscriber_regex = rospy.get_param(
        '~mpc_subscriber_regex',
        r'^/uav\d+/mpc$'
    )
    connection_timeout = float(rospy.get_param('~connection_timeout_sec', 2.0))
    start_delay_sec = float(rospy.get_param('~start_delay_sec', 1.0))
    latch_hold_sec = float(rospy.get_param(
        '~latch_hold_sec',
        rospy.get_param('~publish_duration_sec', 2.0)
    ))
    publish_rate_hz = float(rospy.get_param('~publish_rate_hz', 10.0))

    goal_pub = rospy.Publisher(
        "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
    )

    compiled_mpc_pattern = re.compile(mpc_subscriber_regex)

    def count_mpc_goal_subscribers():
        try:
            _, _, system_state = rospy.get_master().getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[PublishGoal] Failed to query ROS master: %s", exc)
            return 0, []
        subscribers = system_state[1]
        nodes = []
        for topic, topic_nodes in subscribers:
            if topic != "/move_base_simple/goal":
                continue
            nodes.extend(node for node in topic_nodes if compiled_mpc_pattern.match(node))
        return len(set(nodes)), sorted(set(nodes))

    deadline = rospy.Time.now() + rospy.Duration(max(0.0, connection_timeout))
    rate = rospy.Rate(max(1.0, publish_rate_hz))
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        total_ready = goal_pub.get_num_connections() >= max(1, min_subscribers)
        mpc_count, mpc_nodes = count_mpc_goal_subscribers()
        mpc_ready = (
            required_mpc_subscribers <= 0 or
            mpc_count >= required_mpc_subscribers
        )
        if total_ready and mpc_ready:
            break
        rospy.loginfo_throttle(
            2.0,
            "[PublishGoal] Waiting for goal subscribers: total=%d/%d, mpc=%d/%d",
            goal_pub.get_num_connections(),
            max(1, min_subscribers),
            mpc_count,
            max(0, required_mpc_subscribers),
        )
        rate.sleep()

    mpc_count, mpc_nodes = count_mpc_goal_subscribers()
    if required_mpc_subscribers > 0 and mpc_count < required_mpc_subscribers:
        raise RuntimeError(
            "Only {} MPC goal subscribers connected, expected {}. Nodes: {}".format(
                mpc_count,
                required_mpc_subscribers,
                ", ".join(mpc_nodes),
            )
        )

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.position.z = 0.0
    goal.pose.orientation.w = 1.0

    goal.header.stamp = rospy.Time.now() + rospy.Duration(max(0.0, start_delay_sec))
    goal_pub.publish(goal)

    rospy.sleep(max(0.2, latch_hold_sec))
    rospy.loginfo(
        "[PublishGoal] 已发布目标点到 /move_base_simple/goal: x=%.2f, y=%.2f, start_stamp=%.3f, subscribers=%d",
        goal_x,
        goal_y,
        goal.header.stamp.to_sec(),
        goal_pub.get_num_connections(),
    )
    if required_mpc_subscribers > 0:
        rospy.loginfo(
            "[PublishGoal] Confirmed MPC goal subscribers (%d/%d): %s",
            mpc_count,
            required_mpc_subscribers,
            ", ".join(mpc_nodes),
        )


if __name__ == "__main__":
    rospy.init_node("publish_goal", anonymous=False)
    publish_goal()
    rospy.loginfo("[PublishGoal] 目标已发布，节点退出。")
