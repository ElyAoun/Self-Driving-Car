import cv2

class Webcam():
    def __init__(self,captureNumber,screenWidth=320,screenHeight=240):
        self.capture=cv2.VideoCapture(captureNumber)
        self.capture.set(3,screenWidth)
        self.capture.set(4,screenHeight)