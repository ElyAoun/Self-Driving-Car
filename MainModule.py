from LaneDetectionModule import LaneFollower
from MotorModule import Motor
from CarModule import CarModule
from WebcamModule import Webcam
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import math
import numpy as np
import cv2


import cv2
import numpy as np
import random
import logging

def test_video(path):
    # GPIO pins for motor 1
    in1 = 24
    in2 = 23
    # GPIO pins for motor 2
    in3 = 27
    in4 = 22
    # GPIO pins for enable
    en1 = 25
    en2 = 21

    motor1 = Motor(en1, in1, in2)
    motor2 = Motor(en2, in3, in4)
    car = CarModule(motor1, motor2)
    lane_follower = LaneFollower(car)
    cap = cv2.VideoCapture(path + '.mp4')
    try:
        i = 0
        while cap.isOpened():
            _, frame = cap.read()
            print('frame {}'.format(i))
            if frame is not None:
                combo_image = lane_follower.follow_lane(frame)
            else:
                cap.release()
            cv2.imshow("Road with Lane line", combo_image)
            i += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def drive():
    # GPIO pins for motor 1
    in1 = 24
    in2 = 23
    # GPIO pins for motor 2
    in3 = 27
    in4 = 22
    # GPIO pins for enable
    en1 = 25
    en2 = 21

    motor1 = Motor(en1, in1, in2)
    motor2 = Motor(en2, in3, in4)
    car = CarModule(motor1, motor2)
    lane_follower = LaneFollower(car)
    camera_resolution_x=640
    camera_resolution_y=480
    pi_cam = Webcam(camera_resolution_x,camera_resolution_y)
    sleep(0.1)
    for frame in pi_cam.camera.capture_continuous(pi_cam.rawCapture, format="bgr", use_video_port=True):
        image_lane = frame.array
        combo_image = lane_follower.follow_lane(image_lane)
        cv2.imshow("Lane Lines", combo_image)
        key = cv2.waitKey(1) & 0xFF
        pi_cam.rawCapture.truncate(0)
        if key == ord("q"):
            break

if __name__ == '__main__':
    drive()
# test_video("C:\\Users\\jason\\Downloads\\test1")
# test_video("C:\\Users\\jason\\Downloads\\test2")
# test_video("C:\\Users\\jason\\Downloads\\test3")
# test_video("C:\\Users\\jason\\Downloads\\test4")
