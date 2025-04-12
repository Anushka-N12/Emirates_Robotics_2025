import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        # Extract distance measurements from the laser scan data
        ranges = msg.ranges
        front_distance = min(ranges[400:600])  # Front section of the laser data
        threshold_distance = 0.7  # Set a threshold distance to detect obstacles
        twist = Twist()
        if front_distance < threshold_distance:
            # Obstacle detected in front, turn
            twist.angular.z = -1.4  # Turn rate
        else:
            # No obstacle, move forward
            twist.linear.x = 0.5  # Forward speed
        self.publisher.publish(twist)
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    # Create an instance of the ObstacleAvoidance node
    obstacle_avoidance_node = ObstacleAvoidance()
    # Keep the node running and processing callbacks
    rclpy.spin(obstacle_avoidance_node)
    # Shutdown the node and ROS 2 properly
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
