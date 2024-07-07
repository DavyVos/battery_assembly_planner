#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "create_two_step_plan.h"
#include "battery_assembly_interfaces/srv/request_path_planning_execution.hpp"


//convert incoming matrix (16 length float array) to target pose
geometry_msgs::msg::Pose floatArrayToPose(const std::array<double, 16>& double_array) {
    // Ensure the array has exactly 16 elements (already guaranteed by std::array)

    // Reshape the array into a 4x4 matrix
    Eigen::Matrix4f matrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix(i, j) = double_array[i * 4 + j];
        }
    }

    // Extract the position
    Eigen::Vector3f position = matrix.block<3, 1>(0, 3);

    // Extract the rotation matrix
    Eigen::Matrix3f rotation_matrix = matrix.block<3, 3>(0, 0);

    // Convert the rotation matrix to a quaternion
    Eigen::Quaternionf quaternion(rotation_matrix);

    // Create a Pose object
    geometry_msgs::msg::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

void ExecutePlanning(const std::shared_ptr<battery_assembly_interfaces::srv::RequestPathPlanningExecution::Request> request,
          std::shared_ptr<battery_assembly_interfaces::srv::RequestPathPlanningExecution::Response> response,
          std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) 
{
  auto const targetPose = floatArrayToPose(request->transformation_matrix);

  // Set a target Pose with end-effector pointing downwards
  // auto const target_pose = []
  // {
  //   geometry_msgs::msg::Pose msg;

  //   // Set position
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;

  //   // Set orientation for pointing downwards
  //   tf2::Quaternion q;
  //   q.setRPY(-M_PI, 0, 0); // Roll: -pi (180 degrees around X-axis), Pitch: 0, Yaw: 0
  //   msg.orientation = tf2::toMsg(q);

  //   return msg;
  // }();

  // Capture the current (home) position
  auto const home_pose = move_group->getCurrentPose().pose;

  //Execute the planning and respond with failure or success
  bool success = createTwoStepPlan(*move_group, targetPose);
  response->success = success;
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "battery_assembly",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Setup
  static const std::string PLANNING_GROUP = "ur_manipulator";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

  // Create the lambda function that captures move_group
  auto executePlanningCallback = [move_group](
      const std::shared_ptr<battery_assembly_interfaces::srv::RequestPathPlanningExecution::Request> request,
      std::shared_ptr<battery_assembly_interfaces::srv::RequestPathPlanningExecution::Response> response) {
      ExecutePlanning(request, response, move_group);
  };

  // Create the service server using previously created lambda function
  rclcpp::Service<battery_assembly_interfaces::srv::RequestPathPlanningExecution>::SharedPtr service =
      node->create_service<battery_assembly_interfaces::srv::RequestPathPlanningExecution>(
          "request_path_planning_execution",
          executePlanningCallback);

  RCLCPP_INFO(node->get_logger(), "Ready to execute path planning.");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("battery_assembly");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group->getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Spin the node
  rclcpp::spin(node);

  // Shutdown ROS (this will only be reached if rclcpp::spin() is interrupted)
  rclcpp::shutdown();
  return 0;
}