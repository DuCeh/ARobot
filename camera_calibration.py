#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# 摄像头和机械臂的位置校准
import cv2
import numpy as np
import time
import threading
import signal
import LeArm
import kinematics as kin
import RPi.GPIO as GPIO

debug = True

stream = "http://127.0.0.1:8080/?action=stream?dummy=param.mjpg"
cap = cv2.VideoCapture(stream)

orgFrame = None
Running = False

# 校准按键
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
key = 22
GPIO.setup(key, GPIO.IN, GPIO.PUD_UP)
# 校准标志
correction_flag = False

# 暂停信号的回调
def cv_stop(signum, frame):
    global Running

    print("Stop")
    if Running is True:
        Running = False
    cv2.destroyAllWindows()

# 继续信号的回调
def cv_continue(signum, frame):
    global stream
    global Running
    global cap

    print("Continue")
    if Running is False:
        cap = cv2.VideoCapture(stream)
        Running = True

#   注册信号回调
signal.signal(signal.SIGTSTP, cv_stop)
signal.signal(signal.SIGCONT, cv_continue)

# 机械臂位置校准
def Arm_Pos_Corr():
    LeArm.setServo(1, 1200, 500)
    time.sleep(0.5)
    kin.ki_move(0, 2250, 200.0, 1500)

if debug:
    Running = True
else:
    Running = False 

# 运行程序前按下KEY2,进入校准机械臂位置， 校准完成后，再按下KEY退出
run_corr_one = 0
# 初始化机械臂位置
LeArm.runActionGroup('rest', 1)
while True:
    if GPIO.input(key) == 0:
        time.sleep(0.1)
        if GPIO.input(key) == 0:
            correction_flag = not correction_flag
            if correction_flag is False:
                LeArm.runActionGroup('rest', 1)
    if correction_flag is False:
        run_corr_one = 0
        if Running:
          if cap.isOpened():
              ret, orgFrame = cap.read()
              if ret:
                  t1 = cv2.getTickCount()
                  try:             
                      orgFrame = cv2.resize(orgFrame, (320,240), interpolation = cv2.INTER_CUBIC) #将图片缩放到 320*240            
                  except Exception as e:
                      print(e)
                      continue
                  if orgFrame is not None:
                    img_h, img_w = orgFrame.shape[:2]
                    # 画图像中心点
                    cv2.line(orgFrame, (int(img_w / 2) - 20, int(img_h / 2)), (int(img_w / 2) + 20, int(img_h / 2)), (0, 0, 255), 1)
                    cv2.line(orgFrame, (int(img_w / 2),int(img_h / 2) - 20), (int(img_w / 2), int(img_h / 2) + 20), (0, 0, 255), 1)
                    t2 = cv2.getTickCount()
                    time_r = (t2 - t1) / cv2.getTickFrequency()               
                    fps = 1.0/time_r
                    if debug:
                        cv2.putText(orgFrame, "fps:" + str(int(fps)),
                                (10, orgFrame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)#(0, 0, 255)BGR
                        cv2.imshow("orgFrame", orgFrame)
                        cv2.waitKey(1)
                  else:
                    time.sleep(0.01)
    else:
        if correction_flag and run_corr_one == 0:
            run_corr_one += 1
            Arm_Pos_Corr()
        else:
            time.sleep(0.01)
