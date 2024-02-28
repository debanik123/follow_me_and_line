import cv2
import numpy as np
import pyrealsense2 as rs
from easyocr import Reader

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

        # Apply adaptive thresholding
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # _, thresholded = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Perform OCR on the thresholded image
        result = reader.readtext(gray_image, allowlist='0123456789')

        # Display OCR results
        for rectangle_coords, text, confidence in result:
            try:
                if confidence > 0.60:  # Adjust the confidence threshold as needed
                    # Draw rectangle around the detected text
                    cv2.rectangle(color_image, tuple(map(int, rectangle_coords[0])),
                                  tuple(map(int, rectangle_coords[2])), (255, 255, 0), 4)

                    # Display the detected text and confidence
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(color_image, f"Text: {text} (Confidence: {confidence:.2f})",
                                tuple(map(int, rectangle_coords[0])), font, 1.0, (0, 0, 255), 1, cv2.LINE_AA)

                    print(f"Detected text: {text} (Confidence: {confidence:.2f})")
            except Exception as e:
                print(f"Error during OCR: {e}")

        # Display the original frame
        cv2.imshow('RealSense OCR', color_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
