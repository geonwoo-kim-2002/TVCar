#include "calibration/camera_calibration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_node");

    ros::NodeHandle nh;

    Calibration cal;

    Vector4d world;
    Vector3d pixel;

    world << 0, 0, 0, 1;

    // pixel = cal.IntrinsicMatrix() * cal.ExtrinsicMatrix() * world;

    // cout << pixel << "\n\n";
    // cout << cal.IntrinsicMatrix() << "\n\n";
    // cout << cal.ExtrinsicMatrix() << "\n\n";

    ros::Rate spinRate(60);
    while(ros::ok())
    {
        cal.Calibrate();
        cal.showCalImage();
        ros::spinOnce();
        spinRate.sleep();
    }
    return 0;
}