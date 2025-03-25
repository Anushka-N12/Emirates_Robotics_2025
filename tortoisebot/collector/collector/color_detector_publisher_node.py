import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
from ultralytics import YOLO

class ColorDetectorPublisher(Node):

    def __init__(self):
        super().__init__('color_detector_publisher')
        self.publisher_ = self.create_publisher(String, 'color_detection', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # Load YOLOv7 Tiny model
        model = YOLO('../models/yolo11n.pt')

        # Initialize webcam
        cap = cv2.VideoCapture(0)

        color = 'Unknown'
        ret, frame = cap.read()

        # Perform inference
        results = model(frame, save=False, show=False)

        # Ensure results contain valid detections
        if not results[0].boxes:
            print("No objects detected.")
        else:
            largest_area = 0
            largest_box = None

            # Iterate over detected boxes
            for box in results[0].boxes.data:  # Access raw tensor data
                x1, y1, x2, y2, conf, cls = box.tolist()  # Convert tensor to list
                area = (x2 - x1) * (y2 - y1)

                if area > largest_area:
                    largest_area = area
                    largest_box = (x1, y1, x2, y2)  # Store bounding box

            if largest_box:
                x1, y1, x2, y2 = map(int, largest_box)  # Convert to int
                cx = (x1 + x2) // 2  # Center x
                cy = (y1 + y2) // 2  # Center y

                # Convert frame to HSV
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                hue = hsv_frame[cy, cx, 0]  # Get hue value

                if hue < 16:
                    color = 'Red'
                elif hue < 30:
                    color = 'Yellow'
                elif hue > 80 and hue < 160:
                    color = 'Blue'

        # color_str = "Detected color: UNKNOWN"  # Example color detection
        msg = String()
        msg.data = color
        self.get_logger().info(color)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    color_detector_publisher = ColorDetectorPublisher()
    rclpy.spin(color_detector_publisher)
    color_detector_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
