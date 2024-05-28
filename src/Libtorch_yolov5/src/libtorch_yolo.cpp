#include "libtorch_yolo.h"

Libtorch_YOLO::Libtorch_YOLO(ros::NodeHandle node, torch::DeviceType di_type) : detector_("/home/a/yolo_ws/src/20230428_class14.torchscript", di_type)
{
    device_type_ = di_type;
    conf_thres_ = 0.4;
    iou_thres_ = 0.5;
    // check if gpu flag is set

    // set device type - CPU/GPU
    // load class names from dataset for visualization
    class_names_ = LoadNames("/home/a/yolo_ws/src/Libtorch_yolov5/class.names");
    if (class_names_.empty())
    {
        std::cout << "error not class " << std::endl;
    }

    // publish
    //bounding_pub = node.advertise<darknet_ros_msgs::BoundingBox>("/core/yolo", 1000);
    boundings_pub = node.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1000);
    traffic_pub = node.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/traffic_boxes", 1000);

    // subscribe
    subImg = node.subscribe<sensor_msgs::CompressedImage>("/usb_cam1/image_raw", 1, &Libtorch_YOLO::imageCallback, this);
}

std::vector<std::string> Libtorch_YOLO::LoadNames(const std::string &path)
{
    // load class names
    std::vector<std::string> class_names;
    std::ifstream infile(path);

    if (!infile.is_open())
    {
        ROS_ERROR("Error loading the class names!\n");
    }
    else
    {
        std::string name = "";
        while (std::getline(infile, name))
        {
            class_names.push_back(name);
        }
        infile.close();
    }

    return class_names;
}

void Libtorch_YOLO::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    boundingBoxesResults.bounding_boxes.clear();
    trafficBoxesResults.bounding_boxes.clear();
    mImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    mImage = mImagePtr->image;
    // imageHeader_ = msg->header;

    start_time = clock();

    auto result = detector_.Run(mImage, conf_thres_, iou_thres_);
    Demo(mImage, result, class_names_, true);

    cv::namedWindow("Detection", cv::WINDOW_AUTOSIZE);
    
    cv::imshow("Detection", mImage);
    cv::waitKey(1);
}

void Libtorch_YOLO::Demo(cv::Mat &img, const std::vector<std::vector<Detection>> &detections, const std::vector<std::string> &class_names, bool label)
{

    if (detections.empty())
    {
        return;
    }
    int i=0;
    int traffic_id=0;
    darknet_ros_msgs::BoundingBox bb;
    darknet_ros_msgs::BoundingBox traffic_bb;
    for (const auto &detection : detections[0]) 
    {
        const auto &box = detection.bbox;
        float score = detection.score;
        float class_idx = detection.class_idx;

        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

        if (label)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << score;
            s = class_names[class_idx] + " " + ss.str();

            auto font_face = cv::FONT_HERSHEY_DUPLEX;
            auto font_scale = 0.5;
            int thickness = 1;
            int baseline = 0;
            auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);

            cv::rectangle(img, cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
                          cv::Point(box.tl().x + s_size.width, box.tl().y),
                          cv::Scalar(0, 0, 255), -1);

            cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5), font_face, font_scale, cv::Scalar(255, 255, 255), thickness);

            // cv::putText(mImage, "FPS: " + std::to_string(int(1e7 / (clock() - start_time))),
            //             cv::Point(50, 50),
            //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            
        }

        // cv::circle(mImage, cv::Point(bb.xmin, bb.ymin), 5, cv::Scalar(0,255,0), -1);
        // cv::circle(mImage, cv::Point(bb.xmax, bb.ymax), 5, cv::Scalar(255,255,0), -1);
        
        bb.xmin = box.x;
        bb.ymin = box.y;
        bb.xmax = box.x + box.width;
        bb.ymax = box.y + box.height;
        bb.Class = class_names[class_idx];
        bb.probability=score;
        bb.id = i;

        // bounding_pub.publish(bb);
        boundingBoxesResults.bounding_boxes.push_back(bb);
        i++;


        if(class_names[class_idx] != "Car")
        {
            traffic_bb.xmin = box.x;
            traffic_bb.ymin = box.y;
            traffic_bb.xmax = box.x + box.width;
            traffic_bb.ymax = box.y + box.height;
            traffic_bb.Class = class_names[class_idx];
            traffic_bb.probability = score;
            traffic_bb.id = traffic_id;

            trafficBoxesResults.bounding_boxes.push_back(traffic_bb);
            traffic_id++;
        }
    }


    boundingBoxesResults.header.stamp = ros::Time::now();
    boundingBoxesResults.header.frame_id = "detection";
    // boundingBoxesResults.image_header = headerBuff_[(buffIndex_ + 1) % 3];
    boundings_pub.publish(boundingBoxesResults);

    trafficBoxesResults.header.stamp = ros::Time::now();
    trafficBoxesResults.header.frame_id = "detection";
    traffic_pub.publish(trafficBoxesResults);
}
