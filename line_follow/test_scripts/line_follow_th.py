import cv2
import numpy as np
import pyrealsense2 as rs

quit = False
window_name = "D435| 2D View"
key_wait = 10
key = ' '
hsv = None
color_hsv = None

# Create a pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

# Create a window
cv2.namedWindow("Thresholds", cv2.WINDOW_AUTOSIZE)

# Create trackbars
lh, ls, lv = 0, 0, 0
uh, us, uv = 255, 255, 255
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

def on_trackbar_change(val):
    pass

cv2.createTrackbar("LH", "Thresholds", lh, 255, on_trackbar_change)
cv2.createTrackbar("LS", "Thresholds", ls, 255, on_trackbar_change)
cv2.createTrackbar("LV", "Thresholds", lv, 255, on_trackbar_change)
cv2.createTrackbar("UH", "Thresholds", uh, 255, on_trackbar_change)
cv2.createTrackbar("US", "Thresholds", us, 255, on_trackbar_change)
cv2.createTrackbar("UV", "Thresholds", uv, 255, on_trackbar_change)

while True:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()

    # Creating OpenCV Matrix from a color image
    color = np.asanyarray(color_frame.get_data())
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

    hsv = cv2.erode(hsv, None, iterations=2)
    hsv = cv2.dilate(hsv, None, iterations=2)

    # Get trackbar positions
    lh_value = cv2.getTrackbarPos("LH", "Thresholds")
    ls_value = cv2.getTrackbarPos("LS", "Thresholds")
    lv_value = cv2.getTrackbarPos("LV", "Thresholds")
    uh_value = cv2.getTrackbarPos("UH", "Thresholds")
    us_value = cv2.getTrackbarPos("US", "Thresholds")
    uv_value = cv2.getTrackbarPos("UV", "Thresholds")

    # Define the range of color
    color_lower = np.array([lh_value, ls_value, lv_value])
    color_upper = np.array([uh_value, us_value, uv_value])

    color_hsv = cv2.inRange(hsv, color_lower, color_upper)
    color_hsv = cv2.dilate(color_hsv, kernel)

    # Display in a GUI
    cv2.imshow(window_name, color)
    cv2.imshow("hsv", hsv)
    cv2.imshow("color_hsv", color_hsv)
    key = cv2.waitKey(key_wait)

    if key == ord('q'):
        break

cv2.destroyAllWindows()
