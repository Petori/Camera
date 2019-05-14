#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

//int main(int argc,char* argv[])
int main()
{
  Mat srcImage1 = imread("/home/zhsyi/kinova/src/Camera/camera2/src/image/medicine1.png");
  Mat srcImage2 = imread("/home/zhsyi/kinova/src/Camera/camera2/src/image/rectobj.png");
    if (srcImage1.empty()||srcImage2.empty())
    {
        cout << "不能正常加载图片" << endl;
        return -1;
    }

  //使用SURF算子检测关键点
  int minHessian = 400;
  Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);
  vector<KeyPoint>keypoints_object, keypoints_scene;
  detector->detect(srcImage1, keypoints_object);
  detector->detect(srcImage2, keypoints_scene);
    //计算描述符（特征向量）
    Ptr<SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
  Mat descriptors_object, descriptors_scene;
  extractor->compute(srcImage1, keypoints_object, descriptors_object);
  extractor->compute(srcImage2, keypoints_scene, descriptors_scene);
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
  cout<<"最短距离"<<min_dist<<endl;
  cout<<"最大距离"<<max_dist<<endl;
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
  drawMatches(srcImage1, keypoints_object, srcImage2, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


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
/*
  obj_corners[0] = cvPoint(67,62);
  obj_corners[1] = cvPoint(829, 29);
  obj_corners[2] = cvPoint(889, 1194);
  obj_corners[3] = cvPoint(0, 1207);

  obj_corners[0] = cvPoint(170,134);
  obj_corners[1] = cvPoint(498, 169);
  obj_corners[2] = cvPoint(493, 359);
  obj_corners[3] = cvPoint(119, 336);  */
  vector<Point2f>scene_corners(4);
    //进行透视变换
  cout<<obj_corners[0]<<endl;
  perspectiveTransform(obj_corners, scene_corners, H);

  cout<<scene_corners[0]<<endl;

  line(img_matches, scene_corners[0] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[1] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[1] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[2] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[2] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[3] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);
  line(img_matches, scene_corners[3] + Point2f(static_cast<float>(srcImage1.cols), 0), scene_corners[0] + Point2f(static_cast<float>(srcImage1.cols), 0), Scalar(255, 0, 123), 4);

  namedWindow("目标检测结果", WINDOW_NORMAL);
  imshow("目标检测结果", img_matches);
  waitKey(0);
  return 0;

}
