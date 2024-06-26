#include "create_two_step_plan.h"
#include <rclcpp/rclcpp.hpp>

bool createTwoStepPlan(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::msg::Pose& target_pose)
{
    // Step 1: Plan to pre-insertion pose
    geometry_msgs::msg::Pose pre_insertion_pose = target_pose;
    pre_insertion_pose.position.z += 0.1;  // Move 10cm above the target
    
    move_group.setPoseTarget(pre_insertion_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_insertion_plan;
    bool success = (move_group.plan(pre_insertion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (!success)
    {
        RCLCPP_ERROR(rclcpp::get_logger("create_two_step_plan"), "Planning to pre-insertion pose failed");
        return false;
    }
    
    // Execute the pre-insertion plan
    move_group.execute(pre_insertion_plan);
    
    // Step 2: Straight line path to final insertion pose
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group.getCurrentPose().pose);  // Start from current pose
    waypoints.push_back(target_pose);  // End at target pose
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;  // 1cm resolution
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction < 0.95)  // Allow for small tolerance
    {
        RCLCPP_ERROR(rclcpp::get_logger("create_two_step_plan"), "Straight line path planning failed with only %f success", fraction);
        return false;
    }
    
    // Execute the straight line path
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group.execute(cartesian_plan);
    
    return true;
}