#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class RealSensePublisher : public rclcpp::Node {
public:
    RealSensePublisher() : Node("realsense_image_publisher") {
        // Create a RealSense pipeline
        rs2::config cfg;
        rs2::colorizer color_map;

        // Enable the color and depth streams
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        // Start the pipeline
        pipe_.start(cfg);

        // Timer to capture and publish images at a fixed rate (10Hz)
        timer_ = this->create_wall_timer(
            chrono::milliseconds(100),
            std::bind(&RealSensePublisher::publish_image, this)
        );
    }

private:
    void publish_image() {
        // Wait for a coherent pair of frames: depth and color
        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        // Convert RealSense frame to OpenCV matrix
        // Mat color_image(Size(color_frame.as<rs2::video_frame>().get_width(),
        //                      color_frame.as<rs2::video_frame>().get_height()),
        //                 CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        Mat color_image(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);


        // Show the color image in an OpenCV window
        imshow("RealSense Color Image", color_image);

        // Wait for the user to press 'q' to exit
        if (waitKey(1) == 'q') {
            pipe_.stop();
            rclcpp::shutdown();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rs2::pipeline pipe_; // Make the pipeline a class member
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node and spin it
    rclcpp::spin(std::make_shared<RealSensePublisher>());
    
    rclcpp::shutdown();
    return 0;
}
