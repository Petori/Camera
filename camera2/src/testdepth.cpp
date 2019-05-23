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

cv::Mat depthimage;
void image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  //获得深度图的desired and actual数据
    Mat testimage;
    Mat diffimage;
    diffimage.create(depthimage.rows,depthimage.cols,CV_64FC1);
    testimage.create(depthimage.rows,depthimage.cols,CV_64FC1);
    depthimage = cv_bridge::toCvShare(msg)->image;
    imshow(WINDOWS1,depthimage);
    char key1=waitKey(30);
    if(key1 == 32)           //the Ascii of "Space key" is 32
    get_desired = true;


if(get_desired)
{

  get_desired = false;
  ofstream write_desired;
  write_desired.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/srcdesired.txt");
  write_desired<<depthimage;
  write_desired.close();
//exit(0);
  fstream file;
  file.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/srcdesired.txt",ios::in);
  if(file.get()!=EOF)
  {
    cout<<"desired depth_image has been saved"<<endl;
  }
  file.close();

  for(int i = 0;i<depthimage.rows;i++)
  {
    for(int j = 0;j<depthimage.cols;j++)
    {
       testimage.at<double>(i,j) = depthimage.at<ushort>(i,j);
       //testimage.at<double>(i,j)=testimage.at<double>(i,j)/1000;
    }
  }
  ofstream test_desired;
  test_desired.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/testdepth.txt");
  test_desired<<testimage;
  test_desired.close();

  for(int i = 0;i<diffimage.rows;i++)
  {
    for(int j = 0;j<diffimage.cols;j++)
    {
       diffimage.at<double>(i,j) = testimage.at<double>(i,j)- (double)depthimage.at<ushort>(i,j);
       //testimage.at<double>(i,j)=testimage.at<double>(i,j)/1000;
    }
  }
  ofstream diff_desired;
  diff_desired.open("/home/zhsyi/kinova/src/Camera/camera2/src/image/diffdepth.txt");
  diff_desired<<diffimage;
  diff_desired.close();

}
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "testdepth");
  ros::NodeHandle nh;


image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_image = it.subscribe("/camera/depth/image_rect_raw", 1, image_Callback);


  ros::spin();


  return 0;
}
