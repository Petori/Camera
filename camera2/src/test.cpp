#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/core/core.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/core/core.hpp>

#include <ros/spinner.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/subscriber_filter.h>

//PCL头文件
//#include<pcl/io/pcd_io.h>
//#include<pcl/point_cloud.h>
//#include<pcl/visualization/cloud_viewer.h>
//#include <pcl/features/integral_image_normal.h>
//#include<pcl/point_types.h>
//#include<pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include<pcl/common/eigen.h>

#include<pcl/search/search.h>
#include<std_msgs/Int8.h>
#include<common_msgs/targetState.h>
#include<common_msgs/targetsVector.h>
using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;

std::string detect_target_str = "detect_target";
std::string detect_result_str = "detect_result";
ros::Publisher detect_result_pub;
ros::Time timer;

std::string image_rgb_str =  "camera/color/image_raw";
std::string image_depth_str = "camera/depth/image_rect_raw";

cv::Mat mat_image_rgb;
cv::Mat mat_image_depth;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//bool show_image=false;
//bool recognition_on=true;
//common_msgs::targetsVector coordinate_vec;
//define global center point of object
//int32_t* px_py = new int32_t[2];
PointCloud::Ptr obj_cloud(new PointCloud);

//camera parameter
double camera_factor = 1000;
double camera_cx = 633.37;
double camera_cy = 361.89;
double camera_fx = 908.32;
double camera_fy = 908.01;

void GetCloud(cv::Mat image_depth)
{
   obj_cloud->is_dense = false;
   for(int i = 0;i<image_depth.cols;i++)
   {
        for(int j = 0;j<image_depth.rows;j++)
        {
            double d = (double)image_depth.at<ushort>(j,i);
            PointT p;
            p.z = d / camera_factor;
            p.x = (i- camera_cx) * p.z / camera_fx;
            p.y = (j - camera_cy) * p.z / camera_fy;
            cout<<"j="<<j<<"i="<<i<<", z value:"<<p.z<<",    x value:"<<p.x<<endl;
            obj_cloud->points.push_back( p );
        }
    }
    obj_cloud->height = 1;
    obj_cloud->width = obj_cloud->points.size();

    cout<<"number of clouds is :"<<obj_cloud->points.size();

}


void imageCallback_depth( const sensor_msgs::ImageConstPtr  &image_depth )
{
    mat_image_depth = cv_bridge::toCvCopy(image_depth)->image;
   // GetCloud(mat_image_depth);
   // cout<<"3D cloud information has got!"<<endl;
}

void imageCallback_color( const sensor_msgs::ImageConstPtr &image_rgb)
{
      //转换ROS图像消息到opencv图像
        mat_image_rgb = cv_bridge::toCvShare(image_rgb,"bgr8")->image;
        GetCloud(mat_image_depth);
        cout<<"3D cloud information has got!"<<endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "realsense2");
  ros::NodeHandle nh;
  //cv::namedWindow("Show Image");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
/*
  message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(nh, image_rgb_str, 1);
  message_filters::Subscriber<sensor_msgs::Image>image_depth_sub(nh, image_depth_str, 1);

  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_rgb_sub, image_depth_sub, 10);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));
  */
  image_transport::Subscriber sub2 = it.subscribe(image_depth_str, 1, imageCallback_depth);

  image_transport::Subscriber sub1 = it.subscribe(image_rgb_str, 1, imageCallback_color);
/* sleep(1);
    while(ros::ok())
    {
        GetCloud(mat_image_depth);
        sleep(1);
    }  */

//  ros::Subscriber detect_sub = nh.subscribe(detect_target_str, 1000, RobotSignalCallback);
//  detect_result_pub = nh.advertise<common_msgs::targetsVector>(detect_result_str.c_str(), 1000);

  ros::spin();

  return 0;
}
