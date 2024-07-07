#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "create_two_step_plan.h"
#include "battery_assembly_interfaces/srv/request_path_planning_Execution.hpp"


geometry_msgs::Pose floatArrayToPose(const std::array<float, 16>& float_array) {
    // Ensure the array has exactly 16 elements (already guaranteed by std::array)

    // Reshape the array into a 4x4 matrix
    Eigen::Matrix4f matrix;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix(i, j) = float_array[i * 4 + j];
        }
    }

    // Extract the position
    Eigen::Vector3f position = matrix.block<3, 1>(0, 3);

    // Extract the rotation matrix
    Eigen::Matrix3f rotation_matrix = matrix.block<3, 3>(0, 0);

    // Convert the rotation matrix to a quaternion
    Eigen::Quaternionf quaternion(rotation_matrix);

    // Create a Pose object
    geometry_msgs::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
}

void ExecutePlanning(const std::shared_ptr<battery_assembly_interfaces::srv::request_path_planning_Execution::Request> request,
          std::shared_ptr<battery_assembly_interfaces::srv::request_path_planning_Execution::Response> response) 
{
  auto const targetPose = floatArrayToPose(request.transformation_matrix)
}

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "battery_assembly",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("battery_assembly"), "Ready to add two ints.");


  // Setup
  static const std::string PLANNING_GROUP = "ur_manipulator";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("battery_assembly");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group->getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Set a target Pose with end-effector pointing downwards
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;

    // Set position
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;

    // Set orientation for pointing downwards
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, 0); // Roll: -pi (180 degrees around X-axis), Pitch: 0, Yaw: 0
    msg.orientation = tf2::toMsg(q);

    return msg;
  }();

  // Capture the current (home) position
  auto const home_pose = move_group->getCurrentPose().pose;

  bool success = createTwoStepPlan(*move_group, target_pose);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}