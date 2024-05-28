#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <typeinfo>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

#define PI 3.141592

class Calibration
{
private:
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Subscriber image_sub;
    bool is_cloud;
    bool is_image;

    ros::Publisher pcl_pub;
    
    double fx;
    double fy;

    double cx;
    double cy;

    double skew_c;

    double theta_x;
    double theta_y;
    double theta_z;

    double trans_x;
    double trans_y;
    double trans_z;

    pcl::PointCloud<pcl::PointXYZ> pcl_msg;
    vector<Vector3d> cal_pixel;

    cv_bridge::CvImagePtr cv_ptr;

public:
    Calibration();
    MatrixXd RotX(double theta);
    MatrixXd RotY(double theta);
    MatrixXd RotZ(double theta);
    MatrixXd Trans(double x, double y, double z);
    MatrixXd ExtrinsicMatrix(double theta_x, double theta_y, double theta_z, double x, double y, double z);
    MatrixXd IntrinsicMatrix(double fx, double fy, double cx, double cy, double skew_c);
    Vector3d Transformation(Vector4d world);
    void Calibrate();
    void showCalImage();

    void cloud_CB(const sensor_msgs::PointCloud2 &cloud_msg);
    void image_CB(const sensor_msgs::ImageConstPtr& image_msg);
};

#endif