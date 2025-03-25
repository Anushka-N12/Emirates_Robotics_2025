#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
import json


class GoToTrash(Node):
    """
    A class to send a sequence of waypoints to the navigation stack and monitor feedback.

    Attributes:
        _action_client: The ActionClient to send FollowWaypoints goals.
        waypoints (list): A list to store the sequence of waypoints.
        goal (PoseStamped): A PoseStamped object representing the goal pose.
    """

    def __init__(self):
        """Initialize the GoToGoal class."""
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')

        # Initialize a list to store waypoints
        self.waypoints = []

        # Create and append waypoints to the list
        self.load_waypoints_from_file('waypoints.json')

    def load_waypoints_from_file(self, filename):
        """Load waypoints from a JSON file."""
        with open(filename, 'r') as file:
            waypoints_data = json.load(file)
            for waypoint in waypoints_data:
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = "map"
                goal.pose.position.x = waypoint['position']['x']
                goal.pose.position.y = waypoint['position']['y']
                goal.pose.position.z = waypoint['position']['z']
                goal.pose.orientation.x = waypoint['orientation']['x']
                goal.pose.orientation.y = waypoint['orientation']['y']
                goal.pose.orientation.z = waypoint['orientation']['z']
                goal.pose.orientation.w = waypoint['orientation']['w']
                self.waypoints.append(goal)

    def send_goal(self):
        """Send the goal to the action server."""
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        # Wait for the action server to become available
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Register a callback to handle the response from the action server
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal Accepted')

        # Wait for the result from the action server
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info(
            f"Result -> Missed Waypoints: {len(result.missed_waypoints)}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle the feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback -> Current Waypoint: {feedback.current_waypoint}", throttle_duration_sec=2.0)


def main(args=None):
    """Main function to initialize and run the GoToGoal node."""
    rclpy.init(args=args)

    # Create an instance of the GoToGoal class
    gototrash = GoToTrash()

    # Send the goal to the action server
    gototrash.send_goal()

    # Spin the node to process callbacks
    rclpy.spin(gototrash)


if __name__ == '__main__':
    main()
