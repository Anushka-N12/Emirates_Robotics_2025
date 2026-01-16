#include "geometry_msgs/msg/twist.hpp"

class PerformSpin : public BT::SyncActionNode {
public:
  PerformSpin(const std::string &name, const NodeConfiguration &config)
      : BT::SyncActionNode(name, config),
        node_(rclcpp::Node::make_shared("perform_spin")) {
    cmd_vel_publisher_ =
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  NodeStatus tick() override {
    RCLCPP_INFO(node_->get_logger(), "Starting spin");

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 1.0; // Angular velocity

    auto start_time = node_->now();
    while ((node_->now() - start_time).seconds() <
           6.28) // Approximately 360 degrees at 1 rad/s
    {
      cmd_vel_publisher_->publish(cmd);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    cmd.angular.z = 0.0; // Stop spinning
    cmd_vel_publisher_->publish(cmd);

    RCLCPP_INFO(node_->get_logger(), "Spin complete");
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};
