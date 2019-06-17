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
/*
void imageCallback( const sensor_msgs::ImageConstPtr &image_rgb,
                     const sensor_msgs::ImageConstPtr  &image_depth )

{
    cv::Mat mat_image_rgb = cv_bridge::toCvShare(image_rgb)->image;
    cv::Mat mat_image_depth = cv_bridge::toCvShare(image_depth)->image;
    //cout<<4<<endl;
    imshow("show picture1",mat_image_rgb);
    waitKey(0.5);
    //imshow("show picture2",mat_image_depth);
    //waitKey(3);
}
*/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

bool show_image=false;
bool recognition_on=true;
common_msgs::targetsVector coordinate_vec;
//define global center point of object
int32_t* px_py = new int32_t[2];
PointCloud::Ptr obj_cloud(new PointCloud);

//camera parameter
double camera_factor = 1000;
double camera_cx = 633.37;
double camera_cy = 361.89;
double camera_fx = 908.32;
double camera_fy = 908.01;

//sort point by y
bool compy(Point2f &a,Point2f &b)
{
  if (a.y<b.y)
    return true;
  else if (a.y==b.y && a.x<b.x)
    return true;
  else
    return false;
}
bool compx(Point2f &a,Point2f &b)
{
  if (a.x<b.x)
    return true;
  else if (a.x==b.x && a.y<b.y)
    return true;
  else
    return false;
}

void surf_detect(cv::Mat &mat_image_rgb,cv::Rect &rect)
{
  Mat srcImage1 = imread("/home/zhsyi/kinova/src/Camera/camera2/src/image/1.png");

  int minHessian = 400;
  Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);
  vector<KeyPoint>keypoints_object, keypoints_scene;
  detector->detect(srcImage1, keypoints_object);
  detector->detect(mat_image_rgb, keypoints_scene);
    //计算描述符（特征向量）
    Ptr<SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
  Mat descriptors_object, descriptors_scene;
  extractor->compute(srcImage1, keypoints_object, descriptors_object);
  extractor->compute(mat_image_rgb, keypoints_scene, descriptors_scene);
    //使用FLANN匹配算子进行匹配
  FlannBasedMatcher matcher;
  std::vector< DMatch >matches;
  matcher.match(descriptors_object, descriptors_scene, matches);
  double max_dist = 0; double min_dist = 100;
    //计算关键点之间距离的最大值和最小值
  for (int i = 0; i < descriptors_object.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist)min_dist = dist;
    if (dist > max_dist)max_dist = dist;
  }
 // cout<<"最短距离"<<min_dist<<endl;
 // cout<<"最大距离"<<max_dist<<endl;
    //存下匹配距离小于3×min_dist的点对
  std::vector< DMatch >good_matches;
  for (int i = 0; i < descriptors_object.rows; i++)
  {
    if (matches[i].distance < 2 * min_dist)
    {
      good_matches.push_back(matches[i]);
    }
  }
    //绘制出匹配到的关键点
  Mat img_matches;
  drawMatches(srcImage1, keypoints_object, mat_image_rgb, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


  //定义两个局部变量
  vector<Point2f>obj;
  vector<Point2f>scene;

  for (unsigned int i = 0; i < good_matches.size(); i++)
  {
    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);

  }
    //计算透视变换
  Mat H = findHomography(obj, scene, CV_RANSAC);
    //从待测图片中获取角点
  vector<Point2f>obj_corners(4);

  obj_corners[0] = cvPoint(0, 0);
  obj_corners[1] = cvPoint(srcImage1.cols, 0);
  obj_corners[2] = cvPoint(srcImage1.cols, srcImage1.rows);
  obj_corners[3] = cvPoint(0, srcImage1.rows);

  vector<Point2f>scene_corners(4);

    //进行透视变换
  perspectiveTransform(obj_corners, scene_corners, H);

  line(img_matches, scene_corners[0] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[1] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[1] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[2] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[2] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[3] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[3] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[0] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  imwrite("/home/zhsyi/kinova/src/Camera/camera2/src/image/lineobj.png",img_matches);

  vector<Point2f>temp_corners_x=scene_corners;
  vector<Point2f>temp_corners_y=scene_corners;
    Point2f rect_point;
    double rect_height,rect_width;

    sort (temp_corners_x.begin(),temp_corners_x.end(),compx);
    sort (temp_corners_y.begin(),temp_corners_y.end(),compy);
    rect_point.x=temp_corners_x[0].x;
    rect_point.y=temp_corners_y[0].y;

    rect_height=(temp_corners_y.end()-1)->y - (temp_corners_y.begin())->y;
    rect_width=(temp_corners_x.end()-1)->x - (temp_corners_x.begin())->x;

  rect=cv::Rect(rect_point.x,rect_point.y,rect_width,rect_height);
cout<<"rect: "<<rect.x<<",  "<<rect.y<<endl;
  rectangle(mat_image_rgb,rect,Scalar(255,0,0),1,8,0);
  imwrite("/home/zhsyi/kinova/src/Camera/camera2/src/image/rectobj.png",mat_image_rgb);
  //imshow("rectangle",mat_image_rgb);
  //waitKey(0);

}

void GetCloud(cv::Rect &rect, cv::Mat image_rgb, cv::Mat image_depth)
{
  //clouds.clear();
  //px_py.clear();
  //color recognition
  cv::Mat image_HSV;
  std::vector<cv::Mat> HSV_split;
  cv::cvtColor(image_rgb, image_HSV, cv::COLOR_BGR2HSV);
 // cv::split(image_HSV, HSV_split);
  //cv::equalizeHist(HSV_split[2],HSV_split[2]);
 // cv::merge(HSV_split,image_HSV);
  cv::Mat img_thresholded;
  //int minh = 22, maxh = 95, mins = 0, maxs = 255, minv = 31, maxv = 229;
  int minh = 0, maxh = 180, mins = 0, maxs = 255, minv = 0, maxv = 46;
//  int minh = 0, maxh = 10, mins = 43, maxs = 255, minv = 46, maxv = 255;
  cv::inRange(image_HSV, cv::Scalar(minh, mins, minv), cv::Scalar(maxh, maxs, maxv), img_thresholded);

  //开操作 (去除一些噪点)
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_OPEN, element);
  //闭操作 (连接一些连通域)
  cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_CLOSE, element);

    obj_cloud->is_dense = false;/*
    for(int i = rect.x;i<rect.x+rect.width;i++)
    {
      for(int j = rect.y;j<rect.y+rect.height;j++)
      {
          */
          for(int i = 0;i<image_depth.cols;i++)
          {
            for(int j = 0;j<image_depth.rows;j++)
            {
        //remove black area of the image
        //if(img_thresholded.ptr<uchar>(j)[i]>0)
        //  continue;
        // 获取深度图中(i,j)处的值
        double d = (double)image_depth.at<ushort>(j,i);

        d = (double)depthimage.at<ushort>(i,j);
        //if (d>2000)
        //        continue;
        // 计算这个点的空间坐标
        PointT p;
        p.z = d / camera_factor;
        p.x = (i- camera_cx) * p.z / camera_fx;
        p.y = (j - camera_cy) * p.z / camera_fy;
cout<<"j="<<j<<"i="<<i<<", z value:"<<p.z<<",    x value:"<<p.x<<endl;

        // 从rgb图像中获取它的颜色
        // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
        p.b = image_rgb.ptr<uchar>(j)[i*3];
        p.g = image_rgb.ptr<uchar>(j)[i*3+1];
        p.r = image_rgb.ptr<uchar>(j)[i*3+2];

        obj_cloud->points.push_back( p );
      }
    }

    obj_cloud->height = 1;
    obj_cloud->width = obj_cloud->points.size();
    //clouds.push_back(obj_cloud);

    px_py[0] = int32_t(rect.x+rect.width/2);px_py[1] = int32_t(rect.y+rect.height/2);
    //px_py.push_back(px_py);
    cout<<"number of clouds is :"<<obj_cloud->points.size();
    cout<<"center is: "<<px_py[0]<<" , "<<px_py[1]<<endl;
}

void calculate_clouds_coordinate()
{

    coordinate_vec.targets.clear();
    common_msgs::targetState coordinate;
    coordinate.tag=1;
    coordinate.px = px_py[0];
    coordinate.py = px_py[1];


    ////利用PCA主元分析法获得点云的三个主方向，获取质心，计算协方差，获得协方差矩阵，求取协方差矩阵的特征值和特长向量，特征向量即为主方向。
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*obj_cloud, pcaCentroid);
    coordinate.x = pcaCentroid(0);
    coordinate.y = pcaCentroid(1);
    coordinate.z = pcaCentroid(2);
    float coordinate_len = sqrt(coordinate.x*coordinate.x+coordinate.y*coordinate.y+coordinate.z*coordinate.z);
    //if(coordinate_len<1e-5)
    //    continue;
//    ROS_INFO_STREAM("calculate xyz:"<<coordinate.x<<" "<<coordinate.y<<" "<<coordinate.z);

//    ros::Time axis_begin = ros::Time::now();
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*obj_cloud, pcaCentroid, covariance);
//    ROS_INFO_STREAM("computeCovariance!");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//    ros::Time axis_end = ros::Time::now();
//    ros::Duration axis_interval = axis_end-axis_begin;
//    ROS_INFO_STREAM("computing size "<<cloud->points.size()<<" for "<<axis_interval.toSec()<<"s!!!");

    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    Eigen::Vector3f orient0 = eigenVectorsPCA.col(0);
    Eigen::Vector3f orient1 = eigenVectorsPCA.col(1);
    Eigen::Vector3f orient2 = eigenVectorsPCA.col(2);
    float max_orient[3] = {0, 0, 0};
    for(size_t i = 0;i<obj_cloud->points.size();i++)
    {
      Eigen::Vector3f temp_vec;
      temp_vec(0) = obj_cloud->points[i].x-coordinate.x;
      temp_vec(1) = obj_cloud->points[i].y-coordinate.y;
      temp_vec(2) = obj_cloud->points[i].z-coordinate.z;
      float temp_orient0 = abs(temp_vec.dot(orient0));
      float temp_orient1 = abs(temp_vec.dot(orient1));
      float temp_orient2 = abs(temp_vec.dot(orient2));
      max_orient[0] = temp_orient0>max_orient[0]?temp_orient0:max_orient[0];
      max_orient[1] = temp_orient1>max_orient[1]?temp_orient1:max_orient[1];
      max_orient[2] = temp_orient2>max_orient[2]?temp_orient2:max_orient[2];
    }

    std::vector<size_t> idx(3);
    for(size_t i = 0;i!=idx.size();i++)idx[i] = i;
     // 通过比较v的值对索引idx进行排序
    std::sort(idx.begin(), idx.end(), [& max_orient](size_t i1, size_t i2) {return max_orient[i1] < max_orient[i2];});
    Eigen::Matrix3f rotation;
//    for(size_t i = 0;i<3;i++)
    rotation.col(0) = eigenVectorsPCA.col(idx[2]);
    rotation(0,2) = 0;
    rotation(1,2) = 0;
    rotation(2,2) = 1;
    rotation.col(1) = rotation.col(2).cross(rotation.col(0));
    float len = sqrt(rotation(0,1)*rotation(0,1)+rotation(1,1)*rotation(1,1)+rotation(2,1)*rotation(2,1));
    rotation(0,1)/=len;
    rotation(1,1)/=len;
    rotation(2,1)/=len;
    rotation.col(0) = rotation.col(1).cross(rotation.col(2));
    Eigen::Quaternionf quaternion(rotation);
    coordinate.qx = quaternion.x();
    coordinate.qy = quaternion.y();
    coordinate.qz = quaternion.z();
    coordinate.qw = quaternion.w();
    cout<<"center"<<endl;
    cout<< coordinate.x<<endl;
    cout<< coordinate.y<<endl;
    cout<< coordinate.z<<endl;
    //put the calculated coordinate into the vector
    coordinate_vec.targets.push_back(coordinate);

}

void imageCallback_depth( const sensor_msgs::ImageConstPtr  &image_depth )
{
    mat_image_depth = cv_bridge::toCvShare(image_depth)->image;

    //imshow("show picture2",mat_image_depth);
    //waitKey(1.5);
}

void imageCallback_color( const sensor_msgs::ImageConstPtr &image_rgb)
{
      if(recognition_on==true)
      {
        ROS_INFO_STREAM("Recognition is on!!!");

      //转换ROS图像消息到opencv图像
        mat_image_rgb = cv_bridge::toCvShare(image_rgb,"bgr8")->image;

        cv::Rect rect;
        surf_detect(mat_image_rgb,rect);
        cout<<"surf detection has done!"<<endl;

        GetCloud(rect,mat_image_rgb,mat_image_depth);
        cout<<"3D cloud information has got!"<<endl;
/*
        calculate_clouds_coordinate();
        cout<<"PCA method has done!"<<endl;

        //发送点云对应坐标
        detect_result_pub.publish(coordinate_vec);
        cout<<"information publish success!"<<endl;
*/
        if (show_image)
        {
          try
          {
            //cv::imshow(window_rgb_top, cv_bridge::toCvShare(image_rgb)->image);
            cv::imshow("rgb_image", mat_image_rgb);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("Could not convert colored images from '%s' to 'bgr8'.", image_rgb->encoding.c_str());
          }
        }

      }
    //imshow("show picture1",mat_image_rgb);
    //waitKey(1.5);
}


void RobotSignalCallback(const std_msgs::Int8::ConstPtr& msg)
{
  if(msg->data == 1)
    recognition_on = true;
  else
    recognition_on = false;
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

  ros::Subscriber detect_sub = nh.subscribe(detect_target_str, 1000, RobotSignalCallback);
  detect_result_pub = nh.advertise<common_msgs::targetsVector>(detect_result_str.c_str(), 1000);

  ros::spin();

  return 0;
}
