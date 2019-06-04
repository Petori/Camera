/*
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
  Mat image_rgb = imread("/home/zhsyi/camera/src/camera2/src/6.jpg");
  Mat srcImage2 = imread("/home/zhsyi/camera/src/camera2/src/6.jpg");
    if (image_rgb.empty()||srcImage2.empty())
    {
        cout << "不能正常加载图片" << endl;
        return -1;
    }
 imshow("image before handle",image_rgb);
 waitKey(50);
cv::Mat image_HSV;
std::vector<cv::Mat> HSV_split;
cv::cvtColor(image_rgb, image_HSV, cv::COLOR_BGR2HSV);
cv::split(image_HSV, HSV_split);
cv::equalizeHist(HSV_split[2],HSV_split[2]);
cv::merge(HSV_split,image_HSV);
cv::Mat img_thresholded;
int minh = 0, maxh = 180, mins = 0, maxs = 255, minv = 0, maxv = 46;
//  int minh = 0, maxh = 10, mins = 43, maxs = 255, minv = 46, maxv = 255;
cv::inRange(image_HSV, cv::Scalar(minh, mins, minv), cv::Scalar(maxh, maxs, maxv), img_thresholded);

cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_OPEN, element);
//闭操作 (连接一些连通域)
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_CLOSE, element);

imshow("image after handle",img_thresholded);
 waitKey(0);
return 0;
}*/


#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;

/** Global Variables */
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

//! [low]
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
//! [low]

//! [high]
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}

//! [high]
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}

static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}

static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}

static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char* argv[])
{
    //! [cap]
    // VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    //! [cap]

    //! [window]
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    //! [window]

    //! [trackbar]
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    //! [trackbar]

    Mat frame, frame_HSV, frame_threshold;
    while (true) {
        //! [while]
        //cap >> frame;
      frame=imread("/home/zhsyi/kinova/src/Camera/camera2/src/image/things.png");
        if(frame.empty())
        {
            break;
        }

        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        //! [while]

        //! [show]
        // Show the frames
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, element);
        //闭操作 (连接一些连通域)
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_CLOSE, element);
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        //! [show]

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
