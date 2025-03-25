import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpinRobot(Node):

    def __init__(self):
        super().__init__('spin_robot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.spin_duration = 6.28  # Approximate time to spin 360 degrees (2*pi radians)

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        twist = Twist()
        if elapsed_time < self.spin_duration:
            twist.angular.z = 1.0  # Adjust this value to control the spin speed
        else:
            twist.angular.z = 0.0
            self.timer.cancel()  # Stop the timer once the spin is complete

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    spin_robot = SpinRobot()
    rclpy.spin(spin_robot)
    spin_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()