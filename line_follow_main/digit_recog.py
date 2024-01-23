import cv2
import torch
from easyocr import Reader
import numpy as np
import pyrealsense2 as rs

# Initialize EasyOCR reader
reader = Reader(['en'])

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        # Wait for a RealSense frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert RealSense color frame to OpenCV format
        color_image = np.asanyarray(color_frame.get_data())

        # Perform OCR on the image
        result = reader.readtext(color_image)
        # text = result[0][1]
        # print(text)

        # Display the original frame
        cv2.imshow('Original Frame', color_image)

        # # Display OCR results
        for detection in result:
            text = detection
            print(f"Detected text: {text[1]}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
