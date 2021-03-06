
from LaneDetectionModule import LaneFollower
#uncomment on pi
#from MotorModule import Motor
#uncomment on pi
#from CarModule import CarModule
from WebcamModule import Webcam
#uncomment on pi
#import RPi.GPIO as GPIO
from time import sleep
import ClassificationModule
import cv2

detect=0
def test_video(path):
    lane_follower = LaneFollower(None)
    cap = Webcam(path)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 30.0, (int(cap.getCap().get(3)), int(cap.getCap().get(4))))
    stop=False
    try:
        i = 0
        while cap.isOpened():
            _, frame = cap.read()
            #cv2.imwrite("frames\\frame_{}.png".format(i),frame)
            if frame is not None:
                if detect == 1:
                    sign,img_class = ClassificationModule.classify(frame)
                    if sign is not None and sign == 'stop sign':
                        stop = True
                    cv2.imwrite("classifications\\img_{}.png".format(i), img_class)
                combo_image = lane_follower.follow_lane(frame,stop)
                stop=False
                out.write(combo_image)
                #cv2.imshow("Road with Lane line", combo_image)
                #cv2.imwrite("lines\\img_{}.png".format(i),combo_image)

                i += 1
            else:
                break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
#for pi use
"""
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
    cap=cv2.VideoCapture(0)
    sleep(2)
    stop=False
    while True:
        _,image_lane = cap.read()
        if detect==1:
            sign=ClassificationModule.classify(image_lane)
            if sign is not None and sign=='stop sign':
                stop=True
        combo_image = lane_follower.follow_lane(image_lane,stop)
        stop=False
        cv2.imshow("Lane Lines", combo_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    car.stopLR()
if __name__ == '__main__':
    drive()
"""
test_video("in1.mp4")
#test_video("raw_feed.avi")