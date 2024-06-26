#ifndef CREATE_TWO_STEP_PLAN_H
#define CREATE_TWO_STEP_PLAN_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

bool createTwoStepPlan(const rclcpp::Node::SharedPtr& node, const geometry_msgs::msg::Pose& target_pose);

#endif // CREATE_TWO_STEP_PLAN_H