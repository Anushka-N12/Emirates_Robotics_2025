import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class ColorDetectorSub(Node):

    def __init__(self):
        super().__init__('color_detector_sub')
        self.subscription = self.create_subscription(
            String,
            'color_detection',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Detected color: "%s"' % msg.data)

        # Turn right 90 degrees
        twist = Twist()
        twist.angular.z = -1.0  # Adjust this value to control the turn speed
        self.publisher_.publish(twist)
        time.sleep(1.57)  # Adjust this value to control the turn duration

        # Stop
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

        # Turn left 180 degrees
        twist.angular.z = 1.0  # Adjust this value to control the turn speed
        self.publisher_.publish(twist)
        time.sleep(3.14)  # Adjust this value to control the turn duration

        # Stop
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

        # Turn right 90 degrees to return to the original position
        twist.angular.z = -1.0  # Adjust this value to control the turn speed
        self.publisher_.publish(twist)
        time.sleep(1.57)  # Adjust this value to control the turn duration

        # Stop
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    color_detector_sub = ColorDetectorSub()
    rclpy.spin(color_detector_sub)
    color_detector_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()