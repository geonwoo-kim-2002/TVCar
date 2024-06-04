#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime

bridge = CvBridge()
rospack = rospkg.RosPack()
path = rospack.get_path('libtorch_yolov5')
pre_ms = 0

def imageCB(msg):
    global pre_ms
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except (CvBridgeError, e):
        print(e)
    else:
        now = datetime.now()
        if pre_ms > 750000 and now.microsecond < 200000: pre_ms = -200000
        if now.microsecond - pre_ms > 200000:
            pre_ms = now.microsecond
            print(path + '/data/' + str(now.year) + '-' + str(now.month) + '-' + str(now.day) + "_" + str(now.hour) + ":" + str(now.minute) + ":" + str(now.second) + ":" + str(now.microsecond) + '.jpg')
            save_path = path + '/data/' + str(now.year) + '-' + str(now.month) + '-' + str(now.day) + "_" + str(now.hour) + ":" + str(now.minute) + ":" + str(now.second) + ":" + str(now.microsecond) + '.jpg'
            cv2.imwrite(save_path, cv2_img)
            cv2.imshow("Viewer", cv2_img)
            cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node("save_images")

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCB, queue_size=1)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()