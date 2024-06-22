#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import cv2
import numpy as np
import os
import serial
import time
import pyfirmata

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from yoloface.face_detector import YoloDetector
from darknet_ros_msgs.msg import BoundingBox

bridge = CvBridge()
image = Image()
human_box = BoundingBox()
cmd_vel = Twist()
pre_arduino = 's'

def imageCB(msg):
    global image
    image = msg

def yoloCB(msg):
    global human_box
    human_box = msg

def velCB(msg):
    global cmd_vel
    cmd_vel = msg

def detect(model, board):
    global image, human_box, cmd_vel, pre_arduino
    try:
        cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    except (CvBridgeError):
        print("error")
    else:
        height, width, channels = cv2_img.shape

        bboxes,points = model.predict(cv2_img)
        # print(bboxes[0][0][0], bboxes[0][0][1])

        if abs(cmd_vel.angular.z) < 0.1 and abs(cmd_vel.linear.x) < 0.1:
            try:
                if len(bboxes[0]) > 0:
                    x1 = bboxes[0][0][0]
                    y1 = bboxes[0][0][1]
                    x2 = bboxes[0][0][2]
                    y2 = bboxes[0][0][3]
                    for i in range(len(bboxes[0]) - 1):
                        if y2 - y1 < bboxes[0][i + 1][3] - bboxes[0][i + 1][1]:
                            x1 = bboxes[0][i + 1][0]
                            y1 = bboxes[0][i + 1][1]
                            x2 = bboxes[0][i + 1][2]
                            y2 = bboxes[0][i + 1][3]
                    print(bboxes)
                    cv2.rectangle(cv2_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    y_center = (y1 + y2) / 2
                    if y_center >= height / 2 - 50 and y_center <= height / 2 + 50:
                        print("face in center")
                        # stop
                        if pre_arduino == 'd':
                            board.digital[3].write(0)
                            board.digital[4].write(1)
                            board.digital[5].write(1)
                            board.digital[6].write(1)
                            board.digital[7].write(1)
                            time.sleep(0.5)
                        elif pre_arduino == 'u':
                            board.digital[3].write(1)
                            board.digital[4].write(0)
                            board.digital[5].write(1)
                            board.digital[6].write(1)
                            board.digital[7].write(1)
                            time.sleep(0.5)
                        else:
                            board.digital[3].write(1)
                            board.digital[4].write(1)
                            board.digital[5].write(1)
                            board.digital[6].write(1)
                            board.digital[7].write(1)
                            time.sleep(0.5)
                        pre_arduino = 's'
                    elif y_center >= height / 2 + 50:
                        print("face in bottom")
                        # down
                        board.digital[3].write(1)
                        board.digital[4].write(0)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        pre_arduino = 'd'
                    elif y_center <= height / 2 - 50:
                        print("face in top")
                        # up
                        board.digital[3].write(0)
                        board.digital[4].write(1)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        pre_arduino = 'u'
                else:
                    if human_box.ymin < 5:
                        print("not detect and up")
                        # up
                        board.digital[3].write(0)
                        board.digital[4].write(1)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        pre_arduino = 'u'
                    else:
                        print("not detect and down")  
                        # down
                        board.digital[3].write(1)
                        board.digital[4].write(0)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        pre_arduino = 'd'                     

            except:
                print(bboxes)
                print(len(bboxes[0]))
        else:
            print("robot moving")
            # stop
            if pre_arduino == 'd':
                board.digital[3].write(0)
                board.digital[4].write(1)
                board.digital[5].write(1)
                board.digital[6].write(1)
                board.digital[7].write(1)
                time.sleep(0.5)
            elif pre_arduino == 'u':
                board.digital[3].write(1)
                board.digital[4].write(0)
                board.digital[5].write(1)
                board.digital[6].write(1)
                board.digital[7].write(1)
                time.sleep(0.5)
            else:
                board.digital[3].write(1)
                board.digital[4].write(1)
                board.digital[5].write(1)
                board.digital[6].write(1)
                board.digital[7].write(1)
                time.sleep(0.5)
            pre_arduino = 's'
        
        cv2.imshow('detect face', cv2_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("detect_face")

    subimg = rospy.Subscriber("/usb_cam/image_raw1", Image, imageCB, queue_size=1)
    subyolo = rospy.Subscriber("human_box", BoundingBox, yoloCB, queue_size=1)
    subvel = rospy.Subscriber("cmd_vel", Twist, velCB, queue_size=1)

    board = pyfirmata.Arduino('/dev/ttyACM0')

    rospack = rospkg.RosPack()
    path = rospack.get_path('detect_face')

    model = YoloDetector(target_size=640, device="cuda:0", min_face=90)
    
    board.digital[3].write(1)
    board.digital[4].write(1)
    board.digital[5].write(1)
    board.digital[6].write(1)
    board.digital[7].write(1)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        detect(model, board)
        rate.sleep()
