#ifndef LIBTORCH_YOLO_H
#define LIBTORCH_YOLO_H
#include <iostream>
#include <memory>
#include <chrono>
#include <ros/ros.h>
#include <time.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "detector.h"

#include<darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>


class Libtorch_YOLO
{
std::vector<std::string> class_names_;
torch::DeviceType device_type_;


//inpuImg-data
cv::Mat mImage;
cv_bridge::CvImagePtr mImagePtr;

// publish data
darknet_ros_msgs::BoundingBoxes boundingBoxesResults;
darknet_ros_msgs::BoundingBoxes trafficBoxesResults;

// publish && subscribe
ros::Publisher bounding_pub;
ros::Publisher boundings_pub;
ros::Publisher traffic_pub;
ros::Subscriber subImg;

//yolo
Detector detector_;
float conf_thres_;
float iou_thres_;
std::vector<float> data_;


std::string s;
clock_t start_time;
std_msgs::Header headerBuff_[3];
std_msgs::Header imageHeader_;
int buffIndex_ = 0;




public:
    Libtorch_YOLO(ros::NodeHandle node, torch::DeviceType di_type);
    std::vector<std::string> LoadNames(const std::string& path);
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    void Demo(cv::Mat& img,
            const std::vector<std::vector<Detection>>& detections,
            const std::vector<std::string>& class_names,
            bool label);

};



#endif // LIBTORCH_YOLO_H
