#include "calibration/calibration.h"

Calibration::Calibration()
{
    cloud_sub = nh.subscribe("/cloud", 1, &Calibration::cloud_CB, this);
    image_sub = nh.subscribe("/usb_cam/image_raw", 1, &Calibration::image_CB, this);
    yolo_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Calibration::yolo_CB, this);

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("human_cloud", 1);
    human_box_pub = nh.advertise<darknet_ros_msgs::BoundingBox> ("human_box", 1);

    is_cloud = false;
    is_image = false;
    is_yolo = false;

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
        vector<vector<int>> cluster_id;
        vector<int> cluster;
        for(int i = 0;i < cal_pixel.size();i++)
        {
            int x = (int)cal_pixel[i][0];
            int y = (int)cal_pixel[i][1];

            if (pcl_msg.points[i].x > 0)
            {    
                if ((x >= 0 && x < img.cols) && (y >= 0 && y < img.rows))
                {
                    // cout << pcl_msg.points[i].x << ", " << i << endl;
                    int dist_color = (pcl_msg.points[i].x - 0.2) * 100;
                    if(dist_color >= 255) dist_color = 255;
                    
                    // circle(img, Point(img.cols - x, y), 1, CV_RGB(255,0,0));
                    circle(img, Point(x, y), 1, CV_RGB(dist_color, 255 - dist_color, 0));

                    // cout << "angle: " << i / 4.0 - 135 << endl;
                }
                
                double box_center_x = (human_box.xmin + human_box.xmax) / 2;
                double box_margin = 5.0;
                if (human_box.Class == "person" && (x >= human_box.xmin && x <= human_box.xmax) && (y >= human_box.ymin && y <= human_box.ymax))
                {
                    if (cluster.empty())
                    {
                        cluster.push_back(i);
                    }
                    else
                    {
                        int pre_point = cluster[cluster.size() - 1];
                        double dis = std::hypot(pcl_msg.points[i].x - pcl_msg.points[pre_point].x, pcl_msg.points[i].y - pcl_msg.points[pre_point].y);
                        if (dis <= 0.1)
                        {
                            cluster.push_back(i);
                        }
                        else
                        {
                            cluster_id.push_back(cluster);
                            cluster.clear();
                            cluster.push_back(i);
                        }
                    }

                    // cout << pcl_msg.points[i].x << ", " << i << endl;
                    // int dist_color = (pcl_msg.points[i].x - 0.2) * 100;
                    // if(dist_color >= 255) dist_color = 255;
                    
                    // circle(img, Point(img.cols - x, y), 1, CV_RGB(255,0,0));
                    // circle(img, Point(x, y), 1, CV_RGB(dist_color, 255- dist_color, 0));

                    // human_bound.points.push_back(pcl_msg.points[i]);
                    // count++;

                }
            }
        }
        rectangle(img, Point(human_box.xmin, human_box.ymin), Point(human_box.xmax, human_box.ymax), CV_RGB(0, 255, 0), 3);

        if (cluster_id.size() >= 1)
        {
            if (cluster.size() > 0)
            {
                cluster_id.push_back(cluster);
            }
            cout << cluster_id.size() << endl;

            double min_dis = 100;
            int min_id = 0;
            for(int i = 0; i < cluster_id.size();i++)
            {
                int size = cluster_id[i].size();
                double dis = std::hypot(pcl_msg.points[cluster_id[i][(int)(size / 2)]].x, pcl_msg.points[cluster_id[i][(int)(size / 2)]].y);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    min_id = i;
                }
            }
            cout << "min dis: " << min_dis << ", id: " << min_id << endl;
            cout << "size: " << cluster_id[min_id].size() << endl;

            for(int i = 0;i < cluster_id[min_id].size();i++)
            {
                human_bound.points.push_back(pcl_msg.points[cluster_id[min_id][i]]);
                count++;
            }
        }


        std::cout << "\n";
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

void Calibration::yolo_CB(const darknet_ros_msgs::BoundingBoxes &yolo_msg)
{
    is_yolo = true;
    bounding_boxes = yolo_msg;

    std::vector<int> human_idx;
    
    std::cout << "detect object number: " << bounding_boxes.bounding_boxes.size() << std::endl;

    for(int i = 0;i < bounding_boxes.bounding_boxes.size();i++)
    {
        if (bounding_boxes.bounding_boxes[i].Class == "person")
        {
            human_idx.push_back(i);
        }
    }

    if (human_idx.size() == 1)
    {
        human_box = bounding_boxes.bounding_boxes[human_idx[0]];
        pre_human_box = human_box;
    }
    else if (human_idx.size() > 1)
    {
        int min_idx = 0;
        double min_dis = DBL_MAX;

        if (pre_human_box.Class != "")
        {
            int pre_center_x = (pre_human_box.xmin + pre_human_box.xmax) / 2;
            int pre_center_y = (pre_human_box.ymin + pre_human_box.ymax) / 2;

            for(int i = 0;i < human_idx.size();i++)
            {
                int center_x = (bounding_boxes.bounding_boxes[human_idx[i]].xmin + bounding_boxes.bounding_boxes[human_idx[i]].xmax) / 2;
                int center_y = (bounding_boxes.bounding_boxes[human_idx[i]].ymin + bounding_boxes.bounding_boxes[human_idx[i]].ymax) / 2;
                
                double dis = hypot(center_x - pre_center_x, center_y - pre_center_y);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    min_idx = human_idx[i];
                }
            }
        }
        else
        {
            min_idx = human_idx[0];
        }

        human_box = bounding_boxes.bounding_boxes[min_idx];
        pre_human_box = human_box;
    }
    else
    {
        human_box.Class = "";
        pre_human_box.Class = "";
    }

    human_box_pub.publish(human_box);
    // std::cout << "human class: " << human_box.Class << std::endl;
    // if (pre_human_box.Class == "")
    //     std::cout << "pre human class: " << pre_human_box.Class << std::endl;

    // int center_x = (human_box.xmin + human_box.xmax) / 2;
    // int center_y = (human_box.ymin + human_box.ymax) / 2;
    // std::cout << "center x: " << center_x << ", " << "center y: " << center_y << endl; 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_node");

    ros::NodeHandle nh;

    Calibration cal;

    // Vector4d world;
    // Vector3d pixel;

    // world << 0, 0, 0, 1;

    // pixel = cal.IntrinsicMatrix() * cal.ExtrinsicMatrix() * world;

    // cout << pixel << "\n\n";
    // cout << cal.IntrinsicMatrix() << "\n\n";
    // cout << cal.ExtrinsicMatrix() << "\n\n";

    ros::Rate spinRate(30);
    while(ros::ok())
    {
        cal.Calibrate();
        cal.showCalImage();
        ros::spinOnce();
        spinRate.sleep();
    }
    return 0;
}