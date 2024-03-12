import cv2
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np

model = YOLO('/home/dj/Line_follow_DL/src/runs/detect/full_data/weights/best.pt')

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

while True:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    
    if not color_frame:
        continue

    # Convert the color frame to a numpy array
    frame = np.asanyarray(color_frame.get_data())

    midpoint_x, midpoint_y = frame.shape[1] // 2, frame.shape[0] // 2
    new_size = (640, 480)
    frame = cv2.resize(frame[midpoint_y - new_size[1] // 2:midpoint_y + new_size[1] // 2,
                                midpoint_x - new_size[0] // 2:midpoint_x + new_size[0] // 2], new_size)
    # gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Run YOLOv8 inference on the frame
    results = model(frame)
    class_id = int(results[0].boxes.cls.tolist()[0])
    print("class_id --> ",class_id)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Stop streaming
pipeline.stop()
cv2.destroyAllWindows()
