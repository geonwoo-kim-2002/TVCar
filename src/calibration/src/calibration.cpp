#include "calibration/camera_calibration.h"

Calibration::Calibration()
{
    cloud_sub = nh.subscribe("/cloud", 1, &Calibration::cloud_CB, this);
    image_sub = nh.subscribe("/usb_cam/image_raw", 1, &Calibration::image_CB, this);

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("human_cloud", 1);

    is_cloud = false;
    is_image = false;

    ros::NodeHandle private_nh("~");
    private_nh.getParam("fx", fx);
    private_nh.getParam("fy", fy);

    private_nh.getParam("cx", cx);
    private_nh.getParam("cy", cy);

    private_nh.getParam("skew_c", skew_c);

    private_nh.getParam("theta_x", theta_x);
    private_nh.getParam("theta_y", theta_y);
    private_nh.getParam("theta_z", theta_z);

    private_nh.getParam("trans_x", trans_x);
    private_nh.getParam("trans_y", trans_y);
    private_nh.getParam("trans_z", trans_z);
}

MatrixXd Calibration::RotX(double theta)
{
    theta = theta * PI / 180;
    MatrixXd R(3, 3);
    R << 1,          0,             0,
         0, cos(theta), -(sin(theta)),
         0, sin(theta),    cos(theta);
    return R;
}

MatrixXd Calibration::RotY(double theta)
{
    theta = theta * PI / 180;
    MatrixXd R(3, 3);
    R <<    cos(theta), 0, sin(theta),
                     0, 1,          0,
         -(sin(theta)), 0, cos(theta);
    return R;
}

MatrixXd Calibration::RotZ(double theta)
{
    theta = theta * PI / 180;
    MatrixXd R(3, 3);
    R << cos(theta), -(sin(theta)), 0,
         sin(theta),    cos(theta), 0,
                  0,             0, 1;
    return R;
}

MatrixXd Calibration::Trans(double x, double y, double z)
{
    MatrixXd T(3, 4);
    T << 1, 0, 0, x,
         0, 1, 0, y,
         0, 0, 1, z;
    return T;
}

MatrixXd Calibration::ExtrinsicMatrix(double theta_x, double theta_y, double theta_z, double x, double y, double z)
{
    return  RotX(theta_x) * RotY(theta_y) * RotZ(theta_z) * Trans(x, y, z);
}

MatrixXd Calibration::IntrinsicMatrix(double fx, double fy, double cx, double cy, double skew_c)
{
    MatrixXd k(3, 3);
    k << fx, skew_c * fx, cx,
          0,          fy, cy,
          0,           0,  1;
    return k;
}

Vector3d Calibration::Transformation(Vector4d world)
{
    return IntrinsicMatrix(fx, fy, cx, cy, skew_c) * ExtrinsicMatrix(theta_x, theta_y, theta_z, trans_x, trans_y, trans_z) * world;
}

void Calibration::Calibrate()
{
    cal_pixel.clear();
    for(int i = 0;i < pcl_msg.points.size();i++)
    {
        Vector4d pos;
        pos << pcl_msg.points[i].x, pcl_msg.points[i].y, pcl_msg.points[i].z, 1;
        // cout << typeid((double)pcl_msg.points[i].x).name() << ", " << i << endl;

        Vector3d pix = Transformation(pos);
        pix /= pix[2];
        // cout << pix << ", " << i << endl;

        cal_pixel.push_back(pix);
    }
}

void Calibration::showCalImage()
{
    if (is_cloud && is_image)
    {
        Mat img;
        img = cv_ptr->image.clone();

        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        pcl::PointCloud<pcl::PointXYZ> human_bound;
        human_bound.points.clear();
        int count = 0;
        for(int i = 0;i < cal_pixel.size();i++)
        {
            int x = (int)cal_pixel[i][0];
            int y = (int)cal_pixel[i][1];

            if ((x >= 0 && x < img.cols) && (y >= 0 && y < img.rows))
            {
                // cout << pcl_msg.points[i].x << ", " << i << endl;
                int dist_color = (pcl_msg.points[i].x - 0.2) * 800;
                if(dist_color >= 255) dist_color = 255;
                
                // circle(img, Point(img.cols - x, y), 1, CV_RGB(255,0,0));
                circle(img, Point(x, y), 1, CV_RGB(dist_color, 255 - dist_color, 0));

                human_bound.points.push_back(pcl_msg.points[i]);
                count++;
                cout << "angle: " << i / 4.0 - 135 << endl;
            }
            
            // if ((x >= bounding_box && x < bounding_box) && (y >= bounding_box && y < bounding_box))
            // {
            //     // cout << pcl_msg.points[i].x << ", " << i << endl;
            //     int dist_color = (pcl_msg.points[i].x - 0.2) * 800;
            //     if(dist_color >= 255) dist_color = 255;
                
            //     // circle(img, Point(img.cols - x, y), 1, CV_RGB(255,0,0));
            //     circle(img, Point(x, y), 1, CV_RGB(dist_color, 255- dist_color, 0));
            // }
        }
        cout << "\n";
        sensor_msgs::PointCloud2 human_pub;
        pcl::toROSMsg(human_bound, human_pub);
        human_pub.header.frame_id = "cloud";
        human_pub.width = count;
        pcl_pub.publish(human_pub);
        // for (int i = 0; i < human_bound.points.size(); i++)
        // {
        //     std::cout << human_bound.points[i].
        // }

        cv::imshow("OPENCV_WINDOW", img);
        cv::waitKey(2);
    }
}


void Calibration::cloud_CB(const sensor_msgs::PointCloud2 &cloud_msg)
{
    is_cloud = true;

    // pcl::PCLPointCloud2 pcl_pc;
    // pcl_conversions::toPCL(*cloud_msg, pcl_pc);
    // pcl::fromPCLPointCloud2(pcl_pc, pcl_msg);
    pcl::fromROSMsg(cloud_msg, pcl_msg);
    // pcl_pub.publish(pcl_msg);
    // cout << typeid(pcl_msg.points[0]).name() << endl;
}

void Calibration::image_CB(const sensor_msgs::ImageConstPtr& image_msg)
{
    is_image = true;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        // return;
    }
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // Update GUI Window
    // cv::imshow("OPENCV_WINDOW", cv_ptr->image);
    // cout << cv_ptr->image << endl;
    // cv::waitKey(2);
    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
}