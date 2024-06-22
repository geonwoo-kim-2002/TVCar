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
    global image, human_box, cmd_vel
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
                        # arduino.write(b's')   # 아두이노에게  신호 보내기
                        board.digital[3].write(1)
                        board.digital[4].write(1)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        time.sleep(0.1)
                    elif y_center >= height / 2 + 50:
                        print("face in bottom")
                        # arduino.write(b'd')  # 아두이노 시리얼 통신  # 아두이노에게 신호 보내기
                        board.digital[3].write(1)
                        board.digital[4].write(0)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        time.sleep(0.1)
                    elif y_center <= height / 2 - 50:
                        print("face in top")
                        # arduino.write(b'u')
                        board.digital[3].write(0)
                        board.digital[4].write(1)
                        board.digital[5].write(1)
                        board.digital[6].write(1)
                        board.digital[7].write(1)
                        time.sleep(0.1)
                else:
                    if human_box.ymin < 5:
                        print("not detect and up")
                        # arduino.write(b'u')
                    else:
                        print("not detect and down")
                        # arduino.write(b'd')                        

            except:
                print(bboxes)
                print(len(bboxes[0]))
        else:
            print("robot moving")
            # arduino.write(b's')
        
        cv2.imshow('detect face', cv2_img)
        cv2.waitKey(1)
    # k = cv2.waitKey(10) & 0xff  # Press 'ESC' for exiting video
    # if k == 27:
    #     break
    # Cleanup
    # cam.release

if __name__ == '__main__':
    rospy.init_node("detect_face")

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCB, queue_size=1)
    subyolo = rospy.Subscriber("human_box", BoundingBox, yoloCB, queue_size=1)
    subvel = rospy.Subscriber("cmd_vel", Twist, velCB, queue_size=1)

    # 아두이노 시리얼 연결
    # arduino = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노가 연결된 포트. 확인 후 변경 필요.
    board = pyfirmata.Arduino('/dev/ttyACM0')

    rospack = rospkg.RosPack()
    path = rospack.get_path('detect_face')

    # recognizer = cv2.face.LBPHFaceRecognizer_create()
    # recognizer.read('/home/a/capstone/trainer/trainer_0.yml')
    # cascadePath = path + "/data/haarcascade_frontalface_default.xml"
    # print(cascadePath)
    # faceCascade = cv2.CascadeClassifier(cascadePath)
    model = YoloDetector(target_size=640, device="cuda:0", min_face=90)
    
    board.digital[3].write(1)
    board.digital[4].write(1)
    board.digital[5].write(1)
    board.digital[6].write(1)
    board.digital[7].write(1)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        detect(model, board)
        # board.digital[3].write(1)
        # board.digital[4].write(1)
        # board.digital[5].write(1)
        # board.digital[6].write(1)
        # board.digital[7].write(1)
        # time.sleep(1.0)

        # board.digital[3].write(1)
        # board.digital[4].write(0)
        # board.digital[5].write(1)
        # board.digital[6].write(1)
        # board.digital[7].write(1)
        # time.sleep(0.1)
        rate.sleep()

    # arduino.flush()
    # arduino.close()  # 아두이노 연결 종료
    print("\n [INFO] Exiting Program and cleanup stuff")
    