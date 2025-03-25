# filepath: /home/anushka/erc/src/collector/collector/color_test.py
import cv2
from ultralytics import YOLO

# Load YOLOv7 Tiny model
model = YOLO('/home/anushka/Downloads/yolo11n.pt')

# Initialize webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    color = 'Unknown'
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference
    results = model(frame, save=False, show=True)

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

            if hue < 17:
                color = 'Red'
            elif hue < 30:
                color = 'Yellow'
            elif hue > 80 and hue < 160:
                color = 'Blue'
    print(color)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()