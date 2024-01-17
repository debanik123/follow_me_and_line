import cv2
import numpy as np

def yellow_thresholding(input_image):
    lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
    hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
    output_image = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return output_image

def euclidean_distance(point1, point2):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    return np.sqrt(dx * dx + dy * dy)

def minmax_idx(v):
    if not v:
        return None

    min_val, min_index = min((val, idx) for idx, val in enumerate(v))
    max_val, max_index = max((val, idx) for idx, val in enumerate(v))

    return min_val, min_index, max_val, max_index

def all_corners_points(color_image):
    left_up = (5, 5)
    midpoint_left = (5, color_image.shape[0] // 2)
    left_down = (5, color_image.shape[0] - 5)

    mid_left_up_midpoint_left = ((left_up[0]+midpoint_left[0])//2,(left_up[1]+midpoint_left[1])//2)
    mid_left_down_midpoint_left = ((left_down[0]+midpoint_left[0])//2,(left_down[1]+midpoint_left[1])//2)

    midpoint_down = (color_image.shape[1] // 2, color_image.shape[0] - 5)
    right_down = (color_image.shape[1] - 5, color_image.shape[0] - 5)
    midpoint_right = (color_image.shape[1] - 5, color_image.shape[0] // 2)

    right_up = (color_image.shape[1] - 5, 5)
    midpoint_up = (color_image.shape[1] // 2, 5)
    midpoint = (color_image.shape[1] // 2, color_image.shape[0] // 2)

    cv2.circle(color_image, left_up, 5, (0, 0, 255), -1)
    cv2.circle(color_image, midpoint_left, 5, (0, 0, 255), -1)
    cv2.circle(color_image, left_down, 5, (0, 0, 255), -1)
    cv2.circle(color_image, mid_left_up_midpoint_left, 5, (0, 0, 255), -1)
    cv2.circle(color_image, mid_left_down_midpoint_left, 5, (0, 0, 255), -1)
    cv2.circle(color_image, midpoint_down, 5, (0, 0, 255), -1)
    cv2.circle(color_image, right_down, 5, (0, 0, 255), -1)
    cv2.circle(color_image, midpoint_right, 5, (0, 0, 255), -1)
    cv2.circle(color_image, right_up, 5, (0, 0, 255), -1)
    cv2.circle(color_image, midpoint_up, 5, (0, 0, 255), -1)
    cv2.circle(color_image, midpoint, 5, (0, 0, 255), -1)

    p = [left_up, midpoint_left, left_down, mid_left_up_midpoint_left,  mid_left_down_midpoint_left, midpoint_down, right_down, midpoint_right, right_up, midpoint_up, midpoint]
    return p

def draw_circles_on_side(color_image, start_point, end_point, num_circles, circle_radius, circle_color, is_vertical=True):
    if is_vertical:
        step_size = (end_point[1] - start_point[1]) / num_circles
        for i in range(1, num_circles + 1):
            circle_center = (start_point[0], int(start_point[1] + i * step_size))
            cv2.circle(color_image, circle_center, circle_radius, circle_color, -1)
    else:
        step_size = (end_point[0] - start_point[0]) / num_circles
        for i in range(1, num_circles + 1):
            circle_center = (int(start_point[0] + i * step_size), start_point[1])
            cv2.circle(color_image, circle_center, circle_radius, circle_color, -1)


def find_outr_layer(color_image):
    left_up = (5, 5)
    left_down = (5, color_image.shape[0] - 5)
    right_up = (color_image.shape[1] - 5, 5)
    right_down = (color_image.shape[1] - 5, color_image.shape[0] - 5)

    num_circles = 20
    circle_radius = 4
    grid_color = (255, 0, 0)

    draw_circles_on_side(color_image, left_up, left_down, num_circles, circle_radius, grid_color, is_vertical=True)
    draw_circles_on_side(color_image, left_down, right_down, num_circles, circle_radius, grid_color, is_vertical=False)
    draw_circles_on_side(color_image, right_down, right_up, num_circles, circle_radius, grid_color, is_vertical=True)
    draw_circles_on_side(color_image, right_up, left_up, num_circles, circle_radius, grid_color, is_vertical=False)


    # cv2.rectangle(color_image, left_up, right_down, (0, 0, 255), 2)


def find_edge_pixels(edge_image):
    edge_pixels = np.where(edge_image == 255)
    return list(zip(edge_pixels[1], edge_pixels[0]))

def find_midpoints(edge_pixels):
    midpoints = []
    for i in range(0, len(edge_pixels)-1):
        x1, y1 = edge_pixels[i]
        x2, y2 = edge_pixels[i+1]
        midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)
        midpoints.append(midpoint)
    return midpoints

def main():
    # cv2.namedWindow("Yellow Line Following")
    color_image = cv2.imread("follow_line.JPEG")
    color_image = cv2.resize(color_image, (640, 480), interpolation = cv2.INTER_AREA)

    yellow_thresholded = yellow_thresholding(color_image)

    yellow_thresholded = cv2.erode(yellow_thresholded, None, iterations=2)
    yellow_thresholded = cv2.dilate(yellow_thresholded, None, iterations=2)

    edges = cv2.Canny(yellow_thresholded, 50, 150)

    contours, _ = cv2.findContours(yellow_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(color_image, contours, -1, (0, 0, 255), 2)

    # p = all_corners_points(color_image)
    find_outr_layer(color_image)

    
    # edge_pixels = find_edge_pixels(edges)
    # sorted_edge_pixels = sorted(edge_pixels, key=lambda x: x[1], reverse=True)
    # midpoints = find_midpoints(edge_pixels)

    # for mid in midpoints:
    #     cv2.circle(color_image, mid, 1, (0, 255, 0), -1)

    # for i in range(0, len(edge_pixels), 2):
    #     cv2.circle(color_image, sorted_edge_pixels[i], 1, (255, 0, 255), -1)
    
    # print(sorted_edge_pixels)
    # cv2.circle(color_image, sorted_edge_pixels[0], 5, (255, 0, 255), -1)
    # cv2.circle(color_image, sorted_edge_pixels[1], 5, (255, 0, 255), -1)
    
    # cv2.circle(color_image, sorted_edge_pixels[24], 5, (255, 0, 255), -1)
    # cv2.circle(color_image, sorted_edge_pixels[25], 5, (255, 0, 255), -1)

    # cv2.circle(color_image, sorted_edge_pixels[50], 5, (255, 0, 255), -1)
    # cv2.circle(color_image, sorted_edge_pixels[51], 5, (255, 0, 255), -1)

    # cv2.circle(color_image, sorted_edge_pixels[100], 5, (255, 0, 255), -1)
    # cv2.circle(color_image, sorted_edge_pixels[150], 5, (255, 0, 255), -1)
    # cv2.circle(color_image, sorted_edge_pixels[200], 5, (255, 0, 255), -1)

    
    


    


    cv2.imshow("Yellow Line Following", color_image)
    # cv2.imshow("yellow_thresholded", yellow_thresholded)
    cv2.waitKey(0)
    # closing all open windows 
    cv2.destroyAllWindows() 

if __name__ == "__main__":
    main()