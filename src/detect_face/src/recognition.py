# import cv2
# import numpy as np
# import os

# recognizer = cv2.face.LBPHFaceRecognizer_create()
# recognizer.read('/home/a/capstone/trainer/trainer_3.yml')
# cascadePath = "/home/a/capstone/haarcascade_frontalface_default.xml"
# faceCascade = cv2.CascadeClassifier(cascadePath);
# font = cv2.FONT_HERSHEY_SIMPLEX

# #iniciate id counter
# id = 0

# # names related to ids: example ==> loze: id=0,  etc
# # 이런식으로 사용자의 이름을 사용자 수만큼 추가해준다.
# names = ['jiah', 'te']

# # Initialize and start realtime video capture
# cam = cv2.VideoCapture(0)
# cam.set(3, 640) # set video widht
# cam.set(4, 480) # set video height

# # Define min window size to be recognized as a face
# minW = 0.1*cam.get(3)
# minH = 0.1*cam.get(4)

# # roi_x = int(cam.get(3) * 0.25)
# roi_y = int(cam.get(4) * 0.5)
# # roi_width = int(cam.get(3) * 0.5)
# # roi_height = int(cam.get(4) * 0.5)

# # correct = 0
# while True:
#     ret, img =cam.read()
#     # img = cv2.flip(img, -1) # Flip vertically
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
#     faces = faceCascade.detectMultiScale( 
#         gray,
#         scaleFactor = 1.2,
#         minNeighbors = 5,
#         minSize = (int(minW), int(minH)),
#        )

#     for(x,y,w,h) in faces:

#         if y >= roi_y and y <= int(cam.get(4)):
#             # 얼굴이 ROI 안에 있는 경우
#             print("맞다")
            
#         else:
#             print("아니다")

#         cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
#         id, confidence = recognizer.predict(gray[y:y+h,x:x+w])
#         # Check if confidence is less them 100 ==> "0" is perfect match

#         if (confidence < 25) :
#             id = "wait..."
#             confidence = "  {0}%".format(round(100 - confidence))

#         elif (25< confidence < 100):
#             id = names[id]
#             confidence = "  {0}%".format(round(100 - confidence))

#         else:
#             id = "unknown"
#             confidence = "  {0}%".format(round(100 - confidence))
        
#         cv2.putText(img, str(id), (x+5,y-5), font, 1, (255,255,255), 2)
#         cv2.putText(img, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)  
    
#     cv2.imshow('camera',img) 
#     k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video
#     if k == 27:
#         break
# # Do a bit of cleanup
    
# print("\n [INFO] Exiting Program and cleanup stuff")
# cam.release()
# cv2.destroyAllWindows()

#-----------------------------------------------------------------------------

import cv2
import numpy as np
import os

import serial
import time


# 아두이노 시리얼 연결
arduino = serial.Serial('/dev/ttyACM0', 9600)  # 아두이노가 연결된 포트. 확인 후 변경 필요.

recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('/home/a/capstone/trainer/trainer_0.yml')
cascadePath = "/home/a/capstone/haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascadePath)
font = cv2.FONT_HERSHEY_SIMPLEX

# initiate id counter
id = 0

# names related to ids: example ==> loze: id=0,  etc
names = ['jiah', 'None']

# Initialize and start realtime video capture
cam = cv2.VideoCapture(2)
cam.set(3, 640) # set video width
cam.set(4, 480) # set video height

# Define min window size to be recognized as a face
minW = 0.1*cam.get(3)
minH = 0.1*cam.get(4)

roi_y = int(cam.get(4) * 0.5)

while True:
    ret, img = cam.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(int(minW), int(minH)),
    )
    
    for (x, y, w, h) in faces:
        if y >= roi_y and y <= int(cam.get(4)):
            print("맞다")
            arduino.write(b'1')   # 아두이노에게  신호 보내기
            # arduino.flush()
            # time.sleep(1)  # 1초 대기
        else:
            print("아니다")
            arduino.write(b'0')  # 아두이노 시리얼 통신  # 아두이노에게 신호 보내기
            # arduino.flush()
            # time.sleep(1)  # 1초 대기
            
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        id, confidence = recognizer.predict(gray[y:y+h, x:x+w])

        if confidence < 100:
            id = names[id]
            confidence = "  {0}%".format(round(100 - confidence))
        else:
            id = "unknown"
            confidence = "  {0}%".format(round(100 - confidence))
        
        cv2.putText(img, str(id), (x+5, y-5), font, 1, (255, 255, 255), 2)
        cv2.putText(img, str(confidence), (x+5, y+h-5), font, 1, (255, 255, 0), 1)
    
    cv2.imshow('camera', img)
    k = cv2.waitKey(10) & 0xff  # Press 'ESC' for exiting video
    if k == 27:
        break
arduino.flush()
# Cleanup
print("\n [INFO] Exiting Program and cleanup stuff")
arduino.close()  # 아두이노 연결 종료
cam.release

#---------------------------------------------------
# import cv2
# import numpy as np
# import os

# recognizer = cv2.face.LBPHFaceRecognizer_create()
# recognizer.read('/home/a/capstone/trainer/trainer_3.yml')
# cascadePath = "/home/a/capstone/haarcascade_frontalface_default.xml"
# faceCascade = cv2.CascadeClassifier(cascadePath);
# font = cv2.FONT_HERSHEY_SIMPLEX

# # 이름 관련 ID
# names = ['jiah', 'te', 'none']

# # 실시간 비디오 캡처 초기화 및 시작
# cam = cv2.VideoCapture(0)
# cam.set(3, 640) # 비디오 너비 설정
# cam.set(4, 480) # 비디오 높이 설정

# # 얼굴로 인식되는 최소 크기 설정
# minW = 0.1*cam.get(3)
# minH = 0.1*cam.get(4)

# # ROI(관심 영역) 설정
# roi_y = int(cam.get(4) * 0.5)

# while True:
#     ret, img = cam.read()
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
#     faces = faceCascade.detectMultiScale(
#         gray,
#         scaleFactor=1.2,
#         minNeighbors=5,
#         minSize=(int(minW), int(minH)),
#     )

#     # 같은 사람이 여러 번 검출된 경우 가장 높은 퍼센트만 출력하도록 수정
#     face_confidences = {}

#     for (x, y, w, h) in faces:
#         if y >= roi_y and y <= int(cam.get(4)):
#             id, confidence = recognizer.predict(gray[y:y+h, x:x+w])
            
#             # 같은 사람이 여러 번 검출된 경우 가장 높은 퍼센트만 저장
#             if id in face_confidences:
#                 if confidence < face_confidences[id][1]:
#                     face_confidences[id] = (names[id], confidence)
#             else:
#                 face_confidences[id] = (names[id], confidence)

#     # 저장된 가장 높은 퍼센트만 출력
#     for id, (name, confidence) in face_confidences.items():
#         if confidence < 25:
#             name = "wait..."
#             confidence = "  {0}%".format(round(100 - confidence))
#         elif 25 <= confidence < 100:
#             name = names[id]
#             confidence = "  {0}%".format(round(100 - confidence))
#         else:
#             name = "unknown"
#             confidence = "  {0}%".format(round(100 - confidence))
        
#     cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
#     cv2.putText(img, str(name), (x+5, y-5), font, 1, (255, 255, 255), 2)
#     cv2.putText(img, str(confidence), (x+5, y+h-5), font, 1, (255, 255, 0), 1)

#     cv2.imshow('camera', img)
#     k = cv2.waitKey(10) & 0xff
#     if k == 27:
#         break

# print("\n [INFO] Exiting Program and cleanup stuff")
# cam.release()
# cv2.destroyAllWindows()
