#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time
import pyfirmata

import rospy

if __name__ == '__main__':
    rospy.init_node("arduino_test")

    board = pyfirmata.Arduino('/dev/ttyACM0')

    # rospack = rospkg.RosPack()
    # path = rospack.get_path('detect_face')

    # model = YoloDetector(target_size=640, device="cuda:0", min_face=90)
    
    board.digital[3].write(1)
    board.digital[4].write(1)
    board.digital[5].write(1)
    board.digital[6].write(1)
    board.digital[7].write(1)
    time.sleep(0.5)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        board.digital[3].write(1)
        board.digital[4].write(0)
        board.digital[5].write(1)
        board.digital[6].write(1)
        board.digital[7].write(1)
        time.sleep(0.5)
        print("up")
        rate.sleep()
