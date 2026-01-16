import cv2
from ultralytics import YOLO
import numpy as np

# Load YOLOv8 model
model = YOLO(r"C:\Users\anush\Projects\ERC-25\yolo11n.pt")

# Initialize webcam
cap = cv2.VideoCapture(1)

# Keep track of the last detected color
last_color = 'Unknown'

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference
    results = model(frame, save=False, show=True)

    # Ensure results contain valid detections
    if not results[0].boxes:
        print("No objects detected.")
        color = last_color
    else:
        lowest_y2 = -1
        closest_box = None
        print(f'{len(results[0].boxes.data)} objects detected')

        # Iterate over detected boxes to find the closest (lowest y2)
        for box in results[0].boxes.data:  # Access raw tensor data
            x1, y1, x2, y2, conf, cls = box.tolist()  # Convert tensor to list
            if y2 > lowest_y2:  # Find the box with the largest y2 (closest to the camera)
                lowest_y2 = y2
                closest_box = (x1, y1, x2, y2)

        if closest_box:
            x1, y1, x2, y2 = map(int, closest_box)  # Convert to int
            cx = (x1 + x2) // 2  # Center x
            cy = (y1 + y2) // 2  # Center y

            # Enhance saturation of the frame
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            saturation_scale = 2  # Adjust this value to increase/decrease saturation
            h, s, v = cv2.split(hsv_frame)
            s = cv2.multiply(s, saturation_scale)  # Increase saturation
            s = np.clip(s, 0, 255).astype('uint8')  # Ensure values are within [0, 255]
            hsv_enhanced = cv2.merge([h, s, v])
            enhanced_frame = cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)

            # Save the enhanced frame for debugging purposes (optional)
            cv2.imwrite('high_saturation_frame.jpg', enhanced_frame)

            # Define offsets to check four nearby pixels
            offsets = [(-5, -5), (5, -5), (-5, 5), (5, 5)]
            predictions = []

            # Check the hue value at four nearby pixels
            for dx, dy in offsets:
                px, py = cx + dx, cy + dy
                if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:  # Ensure pixel is within bounds
                    hue = hsv_enhanced[py, px, 0]
                    if hue < 15:
                        predictions.append('Red')
                    elif 20 < hue < 35:
                        predictions.append('Yellow')
                    elif 125 < hue < 180:
                        predictions.append('Blue')
                    else:
                        predictions.append('Unknown')

            # Decide on the final color based on predictions
            if predictions.count('Unknown') == len(predictions):
                color = last_color
                print("All predictions are 'Unknown'. Retaining previous color:", last_color)
            else:
                # Count votes for each color
                unique_colors = [c for c in set(predictions) if c != 'Unknown']
                color_counts = {c: predictions.count(c) for c in unique_colors}

                if len(color_counts) == 1:  # Only one valid color
                    color = unique_colors[0]
                else:  # Multiple colors, pick the most voted
                    max_votes = max(color_counts.values())
                    most_voted_colors = [c for c, v in color_counts.items() if v == max_votes]

                    if len(most_voted_colors) == 1:
                        color = most_voted_colors[0]
                    else:
                        color = most_voted_colors[0]  # Pick the first one if there's a tie

    print(color)
    last_color = color

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
