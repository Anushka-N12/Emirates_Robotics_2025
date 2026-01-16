#include "geometry_msgs/msg/pose_stamped.hpp"

// Function to create a PoseStamped for a waypoint
geometry_msgs::msg::PoseStamped createWaypoint(double x, double y, double z,
                                               double yaw) {
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map"; // Reference frame
  waypoint.header.stamp = rclcpp::Clock().now();
  waypoint.pose.position.x = x;
  waypoint.pose.position.y = y;
  waypoint.pose.position.z = z;

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw); // Roll, Pitch, Yaw
  waypoint.pose.orientation.x = q.x();
  waypoint.pose.orientation.y = q.y();
  waypoint.pose.orientation.z = q.z();
  waypoint.pose.orientation.w = q.w();

  return waypoint;
}
int main() {
  rclcpp::init(0, nullptr);

  BehaviorTreeFactory factory;

  factory.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
  factory.registerNodeType<PerformSpin>("PerformSpin");
  factory.registerNodeType<FinalAction>("FinalAction");

  auto tree = factory.createTreeFromText(xml_text);

  // Define waypoints
  auto waypoint_1 =
      createWaypoint(1.0, 1.0, 0.0, 0.0); // x: 1.0, y: 1.0, yaw: 0.0
  auto waypoint_2 =
      createWaypoint(2.0, 2.0, 0.0, 1.57); // x: 2.0, y: 2.0, yaw: 90 degrees
  auto waypoint_3 =
      createWaypoint(3.0, 1.5, 0.0, -1.57); // x: 3.0, y: 1.5, yaw: -90 degrees

  // Set waypoints in the blackboard
  auto blackboard = tree.rootBlackboard();
  blackboard->set("waypoint_1", waypoint_1);
  blackboard->set("waypoint_2", waypoint_2);
  blackboard->set("waypoint_3", waypoint_3);

  // Tick the tree
  tree.tickRoot();

  rclcpp::shutdown();
  return 0;
}
