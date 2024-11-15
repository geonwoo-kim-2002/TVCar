#include <iostream>
#include <memory>
#include <chrono>

#include "libtorch_yolo.h"
#include "detector.h"
#include "cxxopts.hpp"


int main(int argc,char** argv) {


    torch::DeviceType device_;
    if (torch::cuda::is_available()) 
    {
        device_ = torch::kCUDA;
        std::cout << "use GPU" << std::endl;
    } 
    else 
    {
        device_ = torch::kCPU;
        std::cout << "use CPU" << std::endl;
    }

    ros::init(argc,argv,"Libtorch_yolov5");
    ros::NodeHandle node;

    Libtorch_YOLO m_node(node, device_);

    ros::spin();


    return 0;
}
