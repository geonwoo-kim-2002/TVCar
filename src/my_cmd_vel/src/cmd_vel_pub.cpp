#include "my_cmd_vel/cmd_vel_pub.h"

controller::controller(ros::NodeHandle *nh_) : nh(nh_)
{

    odom_subscriber = nh->subscribe("/odom", 1, &controller::messageCallback, this);
    // temp_lidar_subscriber = nh->subscribe("/lidar_object", 10, &controller::templidarCallback, this);
    temp_lidar_subscriber = nh->subscribe("/human_cloud", 10, &controller::templidarCallback, this);
    pre_lidar_subscriber = nh->subscribe("/pre_point", 10, &controller::prelidarCallback, this);
    cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    limit_dist_ = 1.0;

    for (int i = 0; i < 2; i++)
    {
        change_goal_curr_pos_.push_back(0.0);
        curr_vel_.push_back(0.0);
        error_.push_back(0.0);
        gain_.push_back(0.0);
        goal_.push_back(0.0);
        pre_.push_back(0.0);
    }

    gain_.at(0) = 0.38;
    gain_.at(1) = 0.20;
    c = 0.214;
    flag_ = false;
    velocity_ = 0.0;
    steering_ = 0.0;
    pre_velocity_ = 0.0;
    pre_steering_ = 0.0;
    distance_ = 0.0;
    time_ = 0.0;
    human_detect_ = false;
}

void controller::templidarCallback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
    pcl::fromROSMsg(msg, output_pointcloud);
    if (!(output_pointcloud.points.size() == 0))
    {
        // int c_idx = output_pointcloud.size() / 2;
        goal_.at(0) = (output_pointcloud.points.at(0).x + output_pointcloud.points.at(output_pointcloud.points.size() - 1).x) / 2;
        goal_.at(1) = (output_pointcloud.points.at(0).y + output_pointcloud.points.at(output_pointcloud.points.size() - 1).y) / 2;
        distance_ = hypot(goal_.at(0),goal_.at(1));

        flag_ = true;
        human_detect_ = true;

        std::cout << "x: " << goal_.at(0) << ", y: " << goal_.at(1) << std::endl;
    }
    else
    {
        std::cout<<"Human doesn't detect"<<std::endl;
        human_detect_ = false;
    }
}

void controller::prelidarCallback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
    pcl::fromROSMsg(msg, output_pointcloud);
    if (!(output_pointcloud.points.size() == 0))
    {

        pre_.at(0) = output_pointcloud.points.at(0).x;
        pre_.at(1) = output_pointcloud.points.at(0).y;
        flag_ = true;
        
    }
    else
    {
        std::cout<<"Data Doesn't Come"<<std::endl;
    }
}

void controller::messageCallback(const nav_msgs::Odometry &msg)
{
    curr_vel_.at(0) = msg.twist.twist.linear.x;  // m/s
    curr_vel_.at(1) = msg.twist.twist.angular.z; // rad/s
}

void controller ::Saturation(double &vel, double &steering)
{
    if (vel > 1.5)
    {
        vel = 1.5;
    }
    else if (vel < 0)
    {
        vel = 0;
    }

    if (steering > 0.60)
    {
        steering = 0.60;
    }
    else if (steering < -0.60)
    {
        steering = -0.60;
    }
}

void controller::LimitPreInfo(const double &pre_vel, const double &pre_steering, double &vel, double &steering)
{
    if (vel > pre_vel + 1.5)
    {
        vel = pre_vel + 1.5;
    }
    else if (vel < pre_vel - 1.0)
    {
        vel = pre_vel - 1.0;
    }

    if (steering > pre_steering + 0.1)
    {
        steering = pre_steering + 0.1;
    }
    else if (steering < pre_steering - 0.1)
    {
        steering = pre_steering - 0.1;
    }
}

/**
 * @brief this function is main function in this node.
 *
 */
void controller ::process()
{

    vehicle_yaw_ = 0;

    if (flag_)
    {
        geometry_msgs::Twist cmd_vel;

        if (human_detect_)
        {

            change_goal_curr_pos_.at(0) = limit_dist_ * cos(vehicle_yaw_);
            change_goal_curr_pos_.at(1) = limit_dist_ * sin(vehicle_yaw_);

            error_.at(0) = goal_.at(0) - change_goal_curr_pos_.at(0);
            error_.at(1) = goal_.at(1) - change_goal_curr_pos_.at(1);


            velocity_ = cos(vehicle_yaw_) * gain_.at(0) * error_.at(0) + sin(vehicle_yaw_) * gain_.at(1) * error_.at(1);
            // steering_ = 1 / c * (-sin(vehicle_yaw_) * gain_.at(0) * pre_.at(0) + cos(vehicle_yaw_) * gain_.at(1) * pre_.at(1));
            steering_ = 1 / c * (-sin(vehicle_yaw_) * gain_.at(0) * error_.at(0) + cos(vehicle_yaw_) * gain_.at(1) * error_.at(1));


            LimitPreInfo(pre_velocity_, pre_steering_, velocity_, steering_);

            Saturation(velocity_, steering_);

            pre_velocity_ = velocity_;
            pre_steering_ = steering_;

            cmd_vel.linear.x = 2.5 * velocity_;
            cmd_vel.angular.z = steering_;

            // ROS_INFO("object angle : %2f", atan2(error_.at(1), error_.at(0)) * 180 / M_PI);
            // ROS_INFO("vel : %2f", cmd_vel.linear.x);
            // ROS_INFO("ang : %f\n", cmd_vel.angular.z * 180 / M_PI);
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0.5;
        }

        cmd_vel_publisher.publish(cmd_vel);
    }
    else
    {

        std::cout << "Lidar Data Doesn't come" << std::endl;
    }
}
controller::~controller()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_pub");
    ros::NodeHandle nh;

    controller kine(&nh);

    ros::Rate spinRate(60);
    while (ros::ok())
    {
        kine.process();
        ros::spinOnce();
        spinRate.sleep();
    }

    // ros::spin();

    return 0;
}
