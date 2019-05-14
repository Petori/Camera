#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#define WINDOWS1 "depth_image"
using namespace std;
using namespace cv;
bool get_desired=false;

void image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  //获得深度图的desired and actual数据
    cv::Mat depthimage;
    depthimage = cv_bridge::toCvShare(msg)->image;
    imshow(WINDOWS1,depthimage);
    char key1=waitKey(30);
    if(key1 == 32)           //the Ascii of "Space key" is 32
    get_desired = true;


if(get_desired)
{

  get_desired = false;
  ofstream write_desired;
  write_desired.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/desired.txt");
  write_desired<<depthimage;
  write_desired.close();
//exit(0);
  fstream file;
  file.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/desired.txt",ios::in);
  if(file.get()!=EOF)
  {
    cout<<"desired depth_image has been saved"<<endl;
  }
  file.close();

 }

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "testdepth");
  ros::NodeHandle nh;


image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_image = it.subscribe("/camera/depth/image_rect_raw/compressedDepth", 1, image_Callback);


  ros::spin();


  return 0;
}
