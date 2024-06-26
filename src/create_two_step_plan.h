#ifndef CREATE_TWO_STEP_PLAN_H
#define CREATE_TWO_STEP_PLAN_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

bool createTwoStepPlan(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::msg::Pose& target_pose);

#endif // CREATE_TWO_STEP_PLAN_H