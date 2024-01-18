import cv2
import numpy as np
import pyrealsense2 as rs
import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rs_math import PixelToVelocityGenerator_rs, PixeltoPcl

class YellowLineFollower(Node):
    def __init__(self):
        super().__init__('yellow_line_follower')
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.color_map = rs.colorizer()
        self.cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.desiredDistance = 1.2
        self.speed_= 0.85
        self.vel_max = 1.0

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.stop_flag = 0

    def timer_callback(self):
        if self.stop_flag == 1:
            self.stop_robot()



    def start_pipeline(self):
        self.pipe.start(self.cfg)

    def stop_pipeline(self):
        self.pipe.stop()

    def getContourExtent(self, contour):
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)
        rect_area = w*h
        if rect_area > 0:
            return (float(area)/rect_area)

    def yellow_thresholding(self, input_image):
    #     Scalar lower_yellow(0, 0, 255);  // HSV lower limit for yellow
    # Scalar upper_yellow(255, 135, 255);  // HSV upper limit for yellow
        lower_yellow = np.array([0, 50, 115], dtype=np.uint8)
        upper_yellow = np.array([255, 255, 255], dtype=np.uint8)
        hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return yellow_mask
    
    def cluster_create(self, frame, x,y, depth_frame, window_size=5, color_=(255, 0, 255)):
        hm_distances = []
        pixels_min = []
        for i in range(-window_size // 2, window_size // 2 + 1):
            for j in range(-window_size // 2, window_size // 2 + 1):
                current_x = x + i
                current_y = y + j
                pf = PixeltoPcl(depth_frame)
                hm_dis = pf.convert_pixel_to_distance(current_x, current_y)
                if hm_dis:
                    cv2.circle(frame, (current_x, current_y), radius=1, color=color_, thickness=-1)
                    hm_distances.append(hm_dis)
                    pixels_min.append((current_x, current_y))


        min_index, min_distance = min(enumerate(hm_distances), key=lambda x: x[1])
        cv2.circle(frame, pixels_min[min_index], radius=2, color=(0,0,255), thickness=-1)
        min_x, min_y = pixels_min[min_index]

        return (min_x, min_y), min_distance



    def process_frame(self, color_frame, depth_frame):
        color_image = np.asanyarray(color_frame.get_data())
        midpoint_x, midpoint_y = color_image.shape[1] // 2, color_image.shape[0] // 2
        # Define the size of the resized image
        new_size = (color_image.shape[1], 100)
        # Crop and resize the image
        color_image = cv2.resize(color_image[midpoint_y - new_size[1] // 2:midpoint_y + new_size[1] // 2,
                                            midpoint_x - new_size[0] // 2:midpoint_x + new_size[0] // 2], new_size)

        yellow_thresholded = self.yellow_thresholding(color_image)
        yellow_thresholded = cv2.erode(yellow_thresholded, None, iterations=2)
        # yellow_thresholded = cv2.dilate(yellow_thresholded, None, iterations=2)
        contours, _ = cv2.findContours(yellow_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        im_midpoint = (int(color_image.shape[1] // 2.0), int(color_image.shape[0] // 2.0))
        pix_distances = []
        centers = []
        pvg_rs = PixelToVelocityGenerator_rs(depth_frame)

        for contour in contours:
            bounding_box = cv2.boundingRect(contour)
            contour_area = cv2.contourArea(contour)
            # print(contour_area)
            # Set a threshold for the minimum contour area
            min_contour_area_threshold = 1000

            if contour_area > min_contour_area_threshold:
                cv2.rectangle(color_image, bounding_box, (0, 255, 0), 2)
                center= (bounding_box[0] + bounding_box[2] // 2, bounding_box[1] + bounding_box[3] // 2)

                min_cm_pix, min_cm_distance = self.cluster_create(color_image, center[0], center[1], depth_frame, color_=(255, 0, 255))
                min_img_pix, min_Im_distance = self.cluster_create(color_image, im_midpoint[0], im_midpoint[1], depth_frame, color_=(0, 255, 255))

                linear_velocity, angular_velocity, current_distance = pvg_rs.generate_velocity_from_pixels(min_img_pix, min_cm_pix)
                print("Linear Velocity:", linear_velocity, "Angular Velocity:", angular_velocity)

                self.cmd_vel(linear_velocity, angular_velocity)
                
                linear_x_str = "{:.3f}".format(linear_velocity)
                angular_z_str = "{:.3f}".format(angular_velocity)
                curr_dis_str = "{:.4f}".format(current_distance)

                cv2.putText(color_image, "linear_x: "+ linear_x_str +" angular_z: " + angular_z_str,(min_cm_pix[0], min_cm_pix[0]),0, 1.0, (255,255,255),1, lineType=cv2.LINE_AA)
                cv2.putText(color_image, "curr_dis: "+str(curr_dis_str),(min_cm_pix[0], min_cm_pix[0]-80),0, 1.0, (255,255,255),1, lineType=cv2.LINE_AA)
            #     self.stop_flag = 0
            # else:
            #     self.stop_flag = 1
            
        cv2.imshow("Yellow Line Following", color_image)

    def run(self):
        cv2.namedWindow("Yellow Line Following")
        while cv2.waitKey(1) < 0:
            frames = self.pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame and depth_frame:
                try:
                    self.process_frame(color_frame, depth_frame)
                except:
                    pass
            
            rclpy.spin_once(self, timeout_sec=0.0000001)
    
    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def cmd_vel(self, l_v, a_v):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = l_v
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = a_v
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    yellow_line_follower = YellowLineFollower()
    try:
        yellow_line_follower.start_pipeline()
        yellow_line_follower.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        yellow_line_follower.stop_pipeline()
        yellow_line_follower.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
