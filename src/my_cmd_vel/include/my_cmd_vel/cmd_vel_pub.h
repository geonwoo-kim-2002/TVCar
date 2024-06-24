/**
 * @file cmd_vel_pub.h
 * @author Jae Hyoung Yong (jade3011@naver.com)
 * @brief This node make for ICT Challenge for 2023
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <cmath>
#include <vector>

#include <std_msgs/String.h>

using namespace std;

class controller{
private:
    ros::NodeHandle *nh;
    geometry_msgs::Twist cmd_vel;

    ros::Publisher cmd_vel_publisher;
    ros::Subscriber odom_subscriber;

    ros::Subscriber lidar_subscriber;
    ros::Subscriber pre_lidar_subscriber;
    ros::Subscriber temp_lidar_subscriber;

    bool flag_;



    std::vector<double>change_goal_curr_pos_;
    std::vector<double>curr_vel_;
    std::vector<double>error_;
    std::vector<double>gain_;
    std::vector<double>goal_;
    std::vector<double>pre_;


    double c;
    double velocity_;
    double pre_velocity_;
    double pre_steering_;
    double steering_;
    double vehicle_yaw_;

    double limit_dist_;
    double distance_;
    double time_;

    bool human_detect_;

public:
    controller(ros::NodeHandle *nh_);

    void messageCallback(const nav_msgs::Odometry& msg);
    void lidarCallback(const nav_msgs::Odometry &msg);
    void templidarCallback(const sensor_msgs::PointCloud2 &msg);
    void prelidarCallback(const sensor_msgs::PointCloud2 &msg);
    void LimitPreInfo(const double &pre_vel, const double &pre_steering, double & vel, double & steering);
    void Saturation(double &vel, double &steering);
    void process();

    ~controller();
};
