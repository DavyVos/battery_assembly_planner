#ifndef CREATE_TWO_STEP_PLAN_H
#define CREATE_TWO_STEP_PLAN_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


bool createTwoStepPlan(const geometry_msgs::msg::Pose& target_pose);

#endif // CREATE_TWO_STEP_PLAN_H