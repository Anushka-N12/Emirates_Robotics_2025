#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace BT;

class NavigateToWaypoint : public BT::SyncActionNode {
public:
  NavigateToWaypoint(const std::string &name, const NodeConfiguration &config)
      : BT::SyncActionNode(name, config),
        node_(rclcpp::Node::make_shared("navigate_to_waypoint")) {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, "navigate_to_pose");
  }

  static PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }

  NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal)) {
      throw BT::RuntimeError("Missing required input [goal]");
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "NavigateToPose action server not available");
      return NodeStatus::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal_future = client_->async_send_goal(goal_msg);
    rclcpp::spin_until_future_complete(node_, send_goal_future);

    auto result_future = send_goal_future.get()->async_result();
    rclcpp::spin_until_future_complete(node_, result_future);

    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(node_->get_logger(), "Navigation to waypoint failed");
      return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Navigation to waypoint succeeded");
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
};
