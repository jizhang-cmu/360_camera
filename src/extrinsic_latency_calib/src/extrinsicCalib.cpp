#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
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

double minRange = 0.5;
double maxRange = 10.0;
double angAdjustment = 0.01;
double voxelSize = 0.02;
double imageSkipYaw = 0.05;
int imageSkipNum = 4;
int imageSkipCount = 0;
bool is360Cam = true;

int imageWidth = 1920;
int imageHeight = 640;

double kImage[9] = {480.0, 0, 960.5, 0, 480.0, 320.5, 0, 0, 1};
double dImage[4] = {0, 0, 0, 0};

double fx = kImage[0];
double fy = kImage[4];
double cx = kImage[2];
double cy = kImage[5];
double k1 = dImage[0];
double k2 = dImage[1];
double p1 = dImage[2];
double p2 = dImage[3];

Mat mapx, mapy;
Mat kMat, dMat;

pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloudStack(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());

const int odomStackNum = 400;
float lidarXStack[odomStackNum];
float lidarYStack[odomStackNum];
float lidarZStack[odomStackNum];
float lidarRollStack[odomStackNum];
float lidarPitchStack[odomStackNum];
float lidarYawStack[odomStackNum];
double odomTimeStack[odomStackNum];
int odomLastIDPointer = -1;
int odomFrontIDPointer = 0;

double odomTime = 0;
float odomX = 0, odomY = 0, odomZ = 0;

double camRoll = -1.5707963, camPitch = 0, camYaw = -1.5707963;
double camX = 0, camY = 0, camZ = 0;

const int imageStackNum = 10;
Mat imageStack[imageStackNum];
double imageTimeStack[imageStackNum];
int imageLastIDPointer = -1;
int imageFrontIDPointer = 0;

float *depthArray;
pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;

rclcpp::Node::SharedPtr nh;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  odomX = odomIn->pose.pose.position.x;
  odomY = odomIn->pose.pose.position.y;
  odomZ = odomIn->pose.pose.position.z;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  odomLastIDPointer = (odomLastIDPointer + 1) % odomStackNum;
  odomTimeStack[odomLastIDPointer] = odomTime;
  lidarXStack[odomLastIDPointer] = odomX;
  lidarYStack[odomLastIDPointer] = odomY;
  lidarZStack[odomLastIDPointer] = odomZ;
  lidarRollStack[odomLastIDPointer] = roll;
  lidarPitchStack[odomLastIDPointer] = pitch;
  lidarYawStack[odomLastIDPointer] = yaw;
}

void scanHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn)
{
  scanCloud->clear();
  pcl::fromROSMsg(*scanIn, *scanCloud);

  *scanCloudStack += *scanCloud;

  scanCloudCrop->clear();
  int scanCloudStackSize = scanCloudStack->points.size();
  for (int i = 0; i < scanCloudStackSize; i++) {
    float x1 = scanCloudStack->points[i].x - odomX;
    float y1 = scanCloudStack->points[i].y - odomY;
    float z1 = scanCloudStack->points[i].z - odomZ;
    float dis = sqrt(x1 * x1 + y1 * y1 + z1 * z1);

    if (dis < maxRange) scanCloudCrop->push_back(scanCloudStack->points[i]);
  }

  scanCloudStack->clear();
  downSizeFilter.setInputCloud(scanCloudCrop);
  downSizeFilter.filter(*scanCloudStack);
}

void imageHandler(const sensor_msgs::msg::Image::ConstSharedPtr imageIn) 
{
  imageSkipCount--;
  if (imageSkipCount >= 0) return;
  imageSkipCount = imageSkipNum;

  imageLastIDPointer = (imageLastIDPointer + 1) % imageStackNum;
  imageTimeStack[imageLastIDPointer] = rclcpp::Time(imageIn->header.stamp).seconds();;

  cv_bridge::CvImageConstPtr imageInCv = cv_bridge::toCvShare(imageIn, "bgr8");
  if (is360Cam) {
    imageInCv->image.copyTo(imageStack[imageLastIDPointer]);
  } else {
    remap(imageInCv->image, imageStack[imageLastIDPointer], mapx, mapy, CV_INTER_LINEAR);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("extrinsicCalib");

  nh->declare_parameter<double>("minRange", minRange);
  nh->declare_parameter<double>("maxRange", maxRange);
  nh->declare_parameter<double>("angAdjustment", angAdjustment);
  nh->declare_parameter<double>("voxelSize", voxelSize);
  nh->declare_parameter<double>("imageSkipYaw", imageSkipYaw);
  nh->declare_parameter<int>("imageSkipNum", imageSkipNum);
  nh->declare_parameter<double>("camRoll", camRoll);
  nh->declare_parameter<double>("camPitch", camPitch);
  nh->declare_parameter<double>("camYaw", camYaw);
  nh->declare_parameter<double>("camX", camX);
  nh->declare_parameter<double>("camY", camY);
  nh->declare_parameter<double>("camZ", camZ);
  nh->declare_parameter<bool>("is360Cam", is360Cam);
  nh->declare_parameter<int>("imageWidth", imageWidth);
  nh->declare_parameter<int>("imageHeight", imageHeight);
  nh->declare_parameter<double>("fx", fx);
  nh->declare_parameter<double>("fy", fy);
  nh->declare_parameter<double>("cx", cx);
  nh->declare_parameter<double>("cy", cy);
  nh->declare_parameter<double>("k1", k1);
  nh->declare_parameter<double>("k2", k2);
  nh->declare_parameter<double>("p1", p1);
  nh->declare_parameter<double>("p2", p2);

  nh->get_parameter("minRange", minRange);
  nh->get_parameter("maxRange", maxRange);
  nh->get_parameter("angAdjustment", angAdjustment);
  nh->get_parameter("voxelSize", voxelSize);
  nh->get_parameter("imageSkipYaw", imageSkipYaw);
  nh->get_parameter("imageSkipNum", imageSkipNum);
  nh->get_parameter("camRoll", camRoll);
  nh->get_parameter("camPitch", camPitch);
  nh->get_parameter("camYaw", camYaw);
  nh->get_parameter("camX", camX);
  nh->get_parameter("camY", camY);
  nh->get_parameter("camZ", camZ);
  nh->get_parameter("is360Cam", is360Cam);
  nh->get_parameter("imageWidth", imageWidth);
  nh->get_parameter("imageHeight", imageHeight);
  nh->get_parameter("fx", fx);
  nh->get_parameter("fy", fy);
  nh->get_parameter("cx", cx);
  nh->get_parameter("cy", cy);
  nh->get_parameter("k1", k1);
  nh->get_parameter("k2", k2);
  nh->get_parameter("p1", p1);
  nh->get_parameter("p2", p2);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/laser_odometry", 5, odomHandler);

  auto subScan = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 2, scanHandler);

  auto subImage = nh->create_subscription<sensor_msgs::msg::Image>("/camera/image", 2, imageHandler);

  RCLCPP_INFO(nh->get_logger(), "\nPress buttons to adjust camera orientation\n");

  kImage[0] = fx;
  kImage[4] = fy;
  kImage[2] = cx;
  kImage[5] = cy;
  dImage[0] = k1;
  dImage[1] = k2;
  dImage[2] = p1;
  dImage[3] = p2;

  Size imageSize = Size(imageWidth, imageHeight);
  kMat = Mat(3, 3, CV_64FC1, kImage);
  dMat = Mat(4, 1, CV_64FC1, dImage);
  mapx.create(imageSize, CV_32FC1);
  mapy.create(imageSize, CV_32FC1);
  initUndistortRectifyMap(kMat, dMat, Mat(), kMat, imageSize, CV_32FC1, mapx, mapy);

  int imagePixelNum = imageWidth * imageHeight;
  depthArray = new float[imagePixelNum];

  downSizeFilter.setLeafSize(voxelSize, voxelSize, voxelSize);

  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    double imageTime = imageTimeStack[imageFrontIDPointer];
    if (odomLastIDPointer >= 0 && imageLastIDPointer >= 0 && odomTime > imageTime &&
        imageFrontIDPointer != (imageLastIDPointer + 1) % imageStackNum) {
      if (imageTime == 0) {
        imageFrontIDPointer = (imageFrontIDPointer + 1) % imageStackNum;
        waitKey(100);
        status = rclcpp::ok();
        continue;
      }

      Mat image = imageStack[imageFrontIDPointer];
      imageFrontIDPointer = (imageFrontIDPointer + 1) % imageStackNum;

      for (int i = 0; i < imagePixelNum; i++) {
        depthArray[i] = 0;
      }

      while (odomFrontIDPointer != odomLastIDPointer) {
        if (odomTimeStack[odomFrontIDPointer] > imageTime) {
          break;
        }
        odomFrontIDPointer = (odomFrontIDPointer + 1) % odomStackNum;
      }

      bool depthProj = true;
      float lidarRoll = 0, lidarPitch = 0, lidarYaw = 0;
      float lidarX = 0, lidarY = 0, lidarZ = 0;
      if (odomTimeStack[odomFrontIDPointer] < imageTime) {
        lidarX = lidarXStack[odomFrontIDPointer];
        lidarY = lidarYStack[odomFrontIDPointer];
        lidarZ = lidarZStack[odomFrontIDPointer];
        lidarRoll = lidarRollStack[odomFrontIDPointer];
        lidarPitch = lidarPitchStack[odomFrontIDPointer];
        lidarYaw = lidarYawStack[odomFrontIDPointer];
        depthProj = false;
      } else {
        int odomBackIDPointer = (odomFrontIDPointer - 1) % odomStackNum;
        float ratioFront = (imageTime - odomTimeStack[odomBackIDPointer]) 
                         / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);
        float ratioBack = (odomTimeStack[odomFrontIDPointer] - imageTime)
                        / (odomTimeStack[odomFrontIDPointer] - odomTimeStack[odomBackIDPointer]);

        if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] > PI) {
          lidarYawStack[odomBackIDPointer] += 2 * PI;
        } else if (lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer] < -PI) {
          lidarYawStack[odomBackIDPointer] -= 2 * PI;
        }

        lidarX = lidarXStack[odomFrontIDPointer] * ratioFront + lidarXStack[odomBackIDPointer] * ratioBack;
        lidarY = lidarYStack[odomFrontIDPointer] * ratioFront + lidarYStack[odomBackIDPointer] * ratioBack;
        lidarZ = lidarZStack[odomFrontIDPointer] * ratioFront + lidarZStack[odomBackIDPointer] * ratioBack;
        lidarRoll = lidarRollStack[odomFrontIDPointer] * ratioFront + lidarRollStack[odomBackIDPointer] * ratioBack;
        lidarPitch = lidarPitchStack[odomFrontIDPointer] * ratioFront + lidarPitchStack[odomBackIDPointer] * ratioBack;
        lidarYaw = lidarYawStack[odomFrontIDPointer] * ratioFront + lidarYawStack[odomBackIDPointer] * ratioBack;
        
        float deltaYaw = fabs(lidarYawStack[odomFrontIDPointer] - lidarYawStack[odomBackIDPointer]);
        if (deltaYaw > imageSkipYaw) depthProj = false;
      }

      if (depthProj) {
        float sinCamRoll = sin(camRoll);
        float cosCamRoll = cos(camRoll);
        float sinCamPitch = sin(camPitch);
        float cosCamPitch = cos(camPitch);
        float sinCamYaw = sin(camYaw);
        float cosCamYaw = cos(camYaw);

        float sinLidarRoll = sin(lidarRoll);
        float cosLidarRoll = cos(lidarRoll);
        float sinLidarPitch = sin(lidarPitch);
        float cosLidarPitch = cos(lidarPitch);
        float sinLidarYaw = sin(lidarYaw);
        float cosLidarYaw = cos(lidarYaw);

        int scanCloudStackSize = scanCloudStack->points.size();
        for (int i = 0; i < scanCloudStackSize; i++) {
          float x1 = scanCloudStack->points[i].x - lidarX;
          float y1 = scanCloudStack->points[i].y - lidarY;
          float z1 = scanCloudStack->points[i].z - lidarZ;

          float dis = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
          if (dis < minRange || dis > maxRange) continue;

          float x2 = x1 * cosLidarYaw + y1 * sinLidarYaw;
          float y2 = -x1 * sinLidarYaw + y1 * cosLidarYaw;
          float z2 = z1;

          float x3 = x2 * cosLidarPitch - z2 * sinLidarPitch;
          float y3 = y2;
          float z3 = x2 * sinLidarPitch + z2 * cosLidarPitch;

          float x4 = x3;
          float y4 = y3 * cosLidarRoll + z3 * sinLidarRoll;
          float z4 = -y3 * sinLidarRoll + z3 * cosLidarRoll;

          float x5 = x4 - camX;
          float y5 = y4 - camY;
          float z5 = z4 - camZ;

          float x6 = x5 * cosCamYaw + y5 * sinCamYaw;
          float y6 = -x5 * sinCamYaw + y5 * cosCamYaw;
          float z6 = z5;

          float x7 = x6 * cosCamPitch - z6 * sinCamPitch;
          float y7 = y6;
          float z7 = x6 * sinCamPitch + z6 * cosCamPitch;

          float x8 = x7;
          float y8 = y7 * cosCamRoll + z7 * sinCamRoll;
          float z8 = -y7 * sinCamRoll + z7 * cosCamRoll;

          int horiPixelID = -1, vertPixelID = -1;
          float horiDis = sqrt(x8 * x8 + z8 * z8);
          if (is360Cam) {
            horiPixelID = imageWidth / (2 * PI) * atan2(x8, z8) + imageWidth / 2 + 1;
            vertPixelID = imageWidth / (2 * PI) * atan(y8 / horiDis) + imageHeight / 2 + 1;
          } else if (z8 > minRange) {
            horiPixelID = fx * x8 / z8 + cx + 0.5;
            vertPixelID = fy * y8 / z8 + cy + 0.5;
          }
          int pixelVal = 255 * (horiDis - minRange) / (maxRange - minRange);      

          if (horiPixelID >= 1 && horiPixelID < imageWidth - 1 && vertPixelID >= 1 && vertPixelID < imageHeight - 1) {
            for (int ii = -1; ii <= 1; ii++) {
              for (int jj = -1; jj <= 1; jj++) {
                int pixelID = imageWidth * (vertPixelID + ii) + (horiPixelID + jj);
                if (depthArray[pixelID] == 0 || depthArray[pixelID] > horiDis) {
                  image.data[3 * pixelID] = pixelVal;
                  image.data[3 * pixelID + 1] = 255 - pixelVal;
                  depthArray[pixelID] = horiDis;
                }
              }
            }
          }
        }

        imshow("360 Image", image);

        RCLCPP_INFO(nh->get_logger(), "Lidar to camera (xyzypr): %f %f %f %f %f %f\n", camX, camY, camZ, camYaw, camPitch, camRoll);
      } else {
        RCLCPP_INFO(nh->get_logger(), "Skipping frame\n");
      }
    }

    char c = waitKey(100);
    if (c == '1') camRoll -= angAdjustment;
    else if (c == '2') camRoll += angAdjustment;
    else if (c == '3') camPitch -= angAdjustment;
    else if (c == '4') camPitch += angAdjustment;
    else if (c == '5') camYaw -= angAdjustment;
    else if (c == '6') camYaw += angAdjustment;

    status = rclcpp::ok();
  }

  return 0;
}
