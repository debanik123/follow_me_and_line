// include the librealsense C++ header file
#include <librealsense2/rs.hpp>
// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
bool quit = false;
string window_name = "D435| 2D View";
int key_wait = 10;
char key = ' ';
cv::Mat hsv, color_hsv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    rs2::frameset frames;

    // Create a window
    namedWindow("Thresholds", WINDOW_AUTOSIZE);

    // Create trackbars
    int lh = 0, ls = 0, lv = 0;
    int uh = 255, us = 255, uv = 255;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    createTrackbar("LH", "Thresholds",  nullptr, 255);
    createTrackbar("LS", "Thresholds",  nullptr, 255);
    createTrackbar("LV", "Thresholds",  nullptr, 255);
    createTrackbar("UH", "Thresholds",  nullptr, 255);
    createTrackbar("US", "Thresholds",  nullptr, 255);
    createTrackbar("UV", "Thresholds",  nullptr, 255);

    while (true)
    {
        frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        

        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        cvtColor(color, hsv, COLOR_BGR2HSV);

        erode(hsv, hsv, Mat(), Point(-1, -1), 2);
        dilate(hsv, hsv, Mat(), Point(-1, -1), 2);

        // Get trackbar positions
        int lh_value = getTrackbarPos("LH", "Thresholds");
        int ls_value = getTrackbarPos("LS", "Thresholds");
        int lv_value = getTrackbarPos("LV", "Thresholds");
        int uh_value = getTrackbarPos("UH", "Thresholds");
        int us_value = getTrackbarPos("US", "Thresholds");
        int uv_value = getTrackbarPos("UV", "Thresholds");

        // Define the range of color
        Scalar color_lower(lh_value, ls_value, lv_value);
        Scalar color_upper(uh_value, us_value, uv_value);

        inRange(hsv, color_lower, color_upper, color_hsv);
        dilate(color_hsv, color_hsv, kernel);

        // Display in a GUI
        namedWindow(window_name, WINDOW_AUTOSIZE );
        imshow(window_name, color);
        imshow("hsv", hsv);
        imshow("color_hsv", color_hsv);
        key = cv::waitKey(key_wait);

        if (key == 'q') break;
    }
    
    

    

    return 0;
}