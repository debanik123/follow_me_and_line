#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <numeric>
#include <cmath>

using namespace std;
using namespace cv;

double speed_= 0.85;
double stopDistance_ = 0.2;
double desiredDistance = 1.5;
double vel_max = 1.5;

// Function to perform color thresholding for yellow
void yellowThresholding(const Mat& input, Mat& output) {
    Scalar lower_yellow(0, 0, 255);  // HSV lower limit for yellow
    Scalar upper_yellow(255, 135, 255);  // HSV upper limit for yellow
    Mat hsv;
    cvtColor(input, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_yellow, upper_yellow, output);
}

Point2f getContourCenter(const vector<Point>& contour) {
    Moments mu = moments(contour);

    if (mu.m00 == 0) {
        return Point2f(0, 0);
    }

    float x = mu.m10 / mu.m00;
    float y = mu.m01 / mu.m00;

    return Point2f(x, y);
}

std::tuple<float, size_t, float, size_t> minmax_idx(const std::vector<int>& v) {
    if (v.empty()) {
        return {}; // Return an empty tuple for an empty input
    }

    auto minmax_pair = std::minmax_element(v.begin(), v.end());

    float min_val = *minmax_pair.first;
    size_t min_index = static_cast<size_t>(std::distance(v.begin(), minmax_pair.first));

    float max_val = *minmax_pair.second;
    size_t max_index = static_cast<size_t>(std::distance(v.begin(), minmax_pair.second));

    return std::make_tuple(min_val, min_index, max_val, max_index);
}

vector<Point> drawCirclesOnSide(Mat& colorImage, Point start, Point end, int numCircles, int circleRadius, const Scalar& circleColor, bool isVertical = true) {
    float stepSize;
    vector<Point> circleCenters;

    if (isVertical) {
        stepSize = static_cast<float>(end.y - start.y) / numCircles;
        for (int i = 1; i <= numCircles; ++i) {
            Point circleCenter(start.x, static_cast<int>(start.y + i * stepSize));
            circle(colorImage, circleCenter, circleRadius, circleColor, -1);
            circleCenters.push_back(circleCenter);
        }
    } else {
        stepSize = static_cast<float>(end.x - start.x) / numCircles;
        for (int i = 1; i <= numCircles; ++i) {
            Point circleCenter(static_cast<int>(start.x + i * stepSize), start.y);
            circle(colorImage, circleCenter, circleRadius, circleColor, -1);
            circleCenters.push_back(circleCenter);
        }
    }
    return circleCenters;
}


std::vector<Point> findEdgePixels(const Mat& edgeImage) {
    std::vector<Point> edgePixels;

    // Iterate over the image and find non-zero pixels (edge pixels)
    for (int y = 0; y < edgeImage.rows; ++y) {
        for (int x = 0; x < edgeImage.cols; ++x) {
            if (edgeImage.at<uchar>(y, x) == 255) {
                edgePixels.push_back(Point(x, y));
            }
        }
    }

    return edgePixels;
}

float euclideanDistance(const cv::Point& point1, const cv::Point& point2) {
    float dx = static_cast<float>(point2.x - point1.x);
    float dy = static_cast<float>(point2.y - point1.y);
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<cv::Point, cv::Point>> findClosestPoints(const std::vector<cv::Point>& points1, const std::vector<cv::Point>& points2) {
    std::vector<std::pair<cv::Point, cv::Point>> closestPoints;

    for (const auto& point1 : points1) {
        cv::Point closestPoint1;
        cv::Point closestPoint2;
        float minDistance = std::numeric_limits<float>::max();

        for (const auto& point2 : points2) {
            float distance = euclideanDistance(point1, point2);

            if (distance < minDistance) {
                minDistance = distance;
                closestPoint1 = point1;
                closestPoint2 = point2;
            }
        }

        closestPoints.push_back(std::make_pair(closestPoint1, closestPoint2));
    }

    return closestPoints;
}

void visualizeClosestPoints(cv::Mat& color_image, const std::vector<std::pair<cv::Point, cv::Point>>& closestPoints, const cv::Scalar& lineColor, const cv::Scalar& firstCircleColor, const cv::Scalar& secondCircleColor) {
    for (const auto& pair : closestPoints) {
        // Draw line
        cv::line(color_image, pair.first, pair.second, lineColor, 1, cv::LINE_AA);

        // Draw circles at end points
        cv::circle(color_image, pair.first, 4, firstCircleColor, -1);
        cv::circle(color_image, pair.second, 4, secondCircleColor, -1);
    }
}

void drawMidpoints(const std::vector<Point>& edges_pix, cv::Mat& image) {
    for (size_t i = 0; i < edges_pix.size() - 1; i += 2) {
        Point left = edges_pix[i];
        Point right = edges_pix[i + 1];

        int midX = (left.x + right.x) / 2;
        int midY = (left.y + right.y) / 2;

        cv::circle(image, cv::Point(midX, midY), 1, cv::Scalar(0, 0, 255), -1); // Draw a green circle at the midpoint
    }
}

void drawEdgePixels(const std::vector<Point>& edges_pix, cv::Mat& image) {
    for (size_t i = 0; i < edges_pix.size(); i += 1) {
        cv::circle(image, edges_pix[i], 1, Scalar(0, 255, 0), -1); // Draw a green circle at every 20th edge pixel
    }
}

float convertPixelTodepth(rs2::depth_frame& depth, int x, int y) {
    float pixel_distance_in_meters = depth.get_distance(x,y);
    return pixel_distance_in_meters;
}

struct DepthAndPCL {
    float distance;
    float pcl[3];
};

DepthAndPCL convertPixelTo3DWorld(rs2::depth_frame& depth, int x, int y) {
    DepthAndPCL result;

    float upixel[2];
    upixel[0] = static_cast<float>(x);
    upixel[1] = static_cast<float>(y);

    result.distance = depth.get_distance(x, y);
    rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    rs2_deproject_pixel_to_point(result.pcl, &intr, upixel, result.distance);
    //format is Y,Z,X
    // std::cout << "3D Coordinates: (" << result.pcl[0] << ", " << result.pcl[1] << ", " << result.pcl[2] << ")" << std::endl;
    return result;
}

float euclideanDistance(const DepthAndPCL& point1, const DepthAndPCL& point2) {
    float dx = point1.pcl[0] - point2.pcl[0];
    float dy = point1.pcl[1] - point2.pcl[1];
    float dz = point1.pcl[2] - point2.pcl[2];

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Cosin(double a, double b, double c)
{
  return acos(((a*a)+(b*b)-(c*c))/(2*a*b));
}

double gamma_sign_correction(float gamma, float y)
{
    if (y < 0)
    {
        gamma = -abs(gamma);
    }

    if (y > 0)
    {
        gamma = abs(gamma);
    }

    return gamma;
}

double Sqrt(double x, double y)
{
  double h = hypot(x, y);
  return h;
}

geometry_msgs::msg::Twist stop_robot(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub)
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    cmd_vel_pub->publish(cmd_vel_msg);
}

void cmd_vel(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub, double l_v, double a_v)
{
    geometry_msgs::msg::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x =  l_v;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = a_v;

    cmd_vel_pub->publish(cmd_vel_msg);
}


void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub, double x, double y)
{
    double currentDistance = std::sqrt(x * x + y * y);
    double error = abs(desiredDistance - currentDistance);
    double l_v = error*speed_;
    l_v = std::min(l_v, vel_max);
    double a_v = atan(-y/x);
    cmd_vel(cmd_vel_pub, l_v, a_v);
}




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("line_following");

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Declare RealSense pipeline, colorizer, and configuration
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::colorizer color_map;

    // Enable the color and depth streams
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start the pipeline
    pipe.start(cfg);

    // Create a window for visualization
    // namedWindow("Yellow Line Following");

    bool quit = false;
    int key_wait = 10;
    char key = ' ';

    while (!quit) {
        auto frames = pipe.wait_for_frames();
        auto color_frame = frames.get_color_frame();
        auto depth = frames.get_depth_frame();
        Mat color_image(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        

        Mat yellow_thresholded;
        yellowThresholding(color_image, yellow_thresholded);

        // Apply morphological operations to reduce noise
        erode(yellow_thresholded, yellow_thresholded, Mat(), Point(-1, -1), 2);
        dilate(yellow_thresholded, yellow_thresholded, Mat(), Point(-1, -1), 2);

        Mat edges;
        Canny(yellow_thresholded, edges, 240, 255);

        cv::Point leftUp(5, 5);
        cv::Point leftDown(5, color_image.rows - 5);
        cv::Point rightUp(color_image.cols - 5, 5);
        cv::Point rightDown(color_image.cols - 5, color_image.rows - 5);
        int numCircles = 20;

    
        vector<Point> edges_pix = findEdgePixels(edges);
        if(!edges_pix.empty())
        {
            drawEdgePixels(edges_pix, color_image);
            Point midpoint(color_image.cols / 2.0f, color_image.rows-5);
            vector<Point> points_mid {};
            points_mid.push_back(midpoint);

            // convertPixelTo3DWorld(depth_frame, 300, 300);

            std::vector<std::pair<cv::Point, cv::Point>> mid_closestPoints = findClosestPoints(points_mid, edges_pix);
            visualizeClosestPoints(color_image, mid_closestPoints, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255));

            
            for(auto& close_point:mid_closestPoints)
            {
                // float dis_img = convertPixelTodepth(depth, close_point.first.x, close_point.first.y);
                // cout<<"dis_img --> "<<dis_img<<endl;
                // float dis_target = convertPixelTodepth(depth, close_point.second.x, close_point.second.y);
                // cout<<"dis_target --> "<<dis_target<<endl;

                DepthAndPCL img_rs = convertPixelTo3DWorld(depth, close_point.first.x, close_point.first.y);
                DepthAndPCL target_rs = convertPixelTo3DWorld(depth, close_point.second.x, close_point.second.y);

                float distance_img_rs = img_rs.distance;
                float distance_target_rs = target_rs.distance;
                float distance_img_target = euclideanDistance(target_rs, img_rs);
                // Output the calculated distance

                // std::cout << "Distance for target_rs: " << distance_target_rs << " meters" << std::endl;
                // std::cout << "Distance for img_rs: " << distance_img_rs << " meters" << std::endl;
                // std::cout << "Euclidean Distance between img and target: " << distance_img_target << " meters" << std::endl;

                float gamma = gamma_sign_correction(Cosin(distance_target_rs,distance_img_rs, distance_img_target), target_rs.pcl[0]);
                float gpx = distance_target_rs*cos(gamma);
                float gpy = distance_target_rs*sin(gamma);

                if(isfinite(gpx) && isfinite(gpy))
                {
                    double currentDistance = Sqrt(gpx, gpy);
                    move(cmd_vel_pub, gpx, gpy);
                }

                else
                {
                    stop_robot(cmd_vel_pub);
                }

            }

        }
        

        // imshow("Canny Edges", edges);
        imshow("color_image", color_image);
        key = cv::waitKey(key_wait);

        if (key == 'q') quit = true;
        // imshow("Harris Corners", corners);
        rclcpp::spin_some(node);
        
    }

    // Stop the pipeline
    pipe.stop();
    return 0;
}
