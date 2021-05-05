import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
class Webcam():
    def __init__(self,screenWidth=320,screenHeight=240):
        self.camera=PiCamera()
        self.camera.resolution = (screenWidth, screenHeight)
        self.rawCapture = PiRGBArray(self.camera, size=(screenWidth, screenHeight))