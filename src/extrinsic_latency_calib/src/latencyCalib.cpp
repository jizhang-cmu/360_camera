#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.hpp>

using namespace std;
using namespace cv;

const double PI = 3.1415926;

string imu_save_dir;
string image_save_dir;
int resizeImageWidth = 960;
double maxTrackDis = 100;
int boundary = 20;

double timeLast = 0, timeCur = 0;
Mat imageLast, imageCur, imageShow;
vector<Mat> *pyramidsCur = new vector<Mat>();
vector<Mat> *pyramidsLast = new vector<Mat>();
vector<Point2f> featuresLast, featuresCur;
vector<unsigned char> featuresStatus;
vector<float> featuresErrors;

FILE *imuFilePtr = NULL;
FILE *imageFilePtr = NULL;

rclcpp::Node::SharedPtr nh;

void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
  double time = rclcpp::Time(imu->header.stamp).seconds();
  float angRateZ = imu->angular_velocity.z;

  fprintf(imuFilePtr, "%f %f\n", angRateZ, time);
  fflush(imuFilePtr);
}

void imageHandler(const sensor_msgs::msg::Image::ConstSharedPtr imageIn) 
{
  timeLast = timeCur;
  timeCur = rclcpp::Time(imageIn->header.stamp).seconds();

  Mat imageTemp = imageLast;
  imageLast = imageCur;
  imageCur = imageTemp;

  cv_bridge::CvImageConstPtr imageInCv = cv_bridge::toCvShare(imageIn, "mono8");
  int resizeImageHeight = resizeImageWidth * float(imageInCv->image.size().height) / float(imageInCv->image.size().width);
  resize(imageInCv->image, imageCur, Size(resizeImageWidth, resizeImageHeight));
  cvtColor(imageCur, imageShow, CV_GRAY2BGR);

  vector<Mat> *pyramidsTemp = pyramidsLast;
  pyramidsLast = pyramidsCur;
  pyramidsCur = pyramidsTemp;

  buildOpticalFlowPyramid(imageCur, *pyramidsCur, Size(15, 15), 3, 1);

  if (timeLast == 0 || timeCur == 0) return;

  goodFeaturesToTrack(imageLast, featuresLast, 500, 0.1, 5.0);
  calcOpticalFlowPyrLK(*pyramidsLast, *pyramidsCur, featuresLast, featuresCur, featuresStatus, featuresErrors,
                       Size(15, 15), 3, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), 0, 1e-4);

  int validNum = 0;
  float horiShift = 0;
  int featureNum = featuresCur.size();
  for (int i = 0; i < featureNum; i++) {
    float trackDis = sqrt((featuresLast[i].x - featuresCur[i].x) * (featuresLast[i].x - featuresCur[i].x)
                   + (featuresLast[i].y - featuresCur[i].y) * (featuresLast[i].y - featuresCur[i].y));

    if (!(trackDis > maxTrackDis || featuresCur[i].x < boundary || 
      featuresCur[i].x > resizeImageWidth - boundary || featuresCur[i].y < boundary || 
      featuresCur[i].y > resizeImageHeight - boundary) && featuresStatus[i]) {

      horiShift += featuresCur[i].x - featuresLast[i].x;
      validNum++;
      
      line(imageShow, Point(featuresLast[i].x, featuresLast[i].y), Point(featuresCur[i].x, featuresCur[i].y), CV_RGB(255, 0, 0), 1);
      circle(imageShow, Point(featuresCur[i].x, featuresCur[i].y), 1, CV_RGB(255, 0, 0), 2);
    }
  }

  if (validNum > 0) {
    horiShift /= validNum;
    fprintf(imageFilePtr, "%f %f\n", horiShift, (timeLast + timeCur) / 2);
    fflush(imageFilePtr);
  }

  imshow("Image", imageShow);
  waitKey(10);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("latencyCalib");

  nh->declare_parameter<std::string>("imu_save_dir", imu_save_dir);
  nh->declare_parameter<std::string>("image_save_dir", image_save_dir);
  nh->declare_parameter<double>("maxTrackDis", maxTrackDis);
  nh->declare_parameter<int>("boundary", boundary);

  nh->get_parameter("imu_save_dir", imu_save_dir);
  nh->get_parameter("image_save_dir", image_save_dir);
  nh->get_parameter("resizeImageWidth", resizeImageWidth);
  nh->get_parameter("maxTrackDis", maxTrackDis);
  nh->get_parameter("boundary", boundary);

  imu_save_dir.replace(imu_save_dir.find("/install/"), 8, "/src");
  image_save_dir.replace(image_save_dir.find("/install/"), 8, "/src");

  auto subIMU = nh->create_subscription<sensor_msgs::msg::Imu> ("/imu/data", 5, imuHandler);

  auto subImage = nh->create_subscription<sensor_msgs::msg::Image>("/camera/image", 2, imageHandler);

  imuFilePtr = fopen(imu_save_dir.c_str(), "w");
  imageFilePtr = fopen(image_save_dir.c_str(), "w");

  rclcpp::spin(nh);

  fclose(imuFilePtr);
  fclose(imageFilePtr);

  return 0;
}
