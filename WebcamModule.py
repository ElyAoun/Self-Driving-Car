import cv2


class Webcam():

    def __init__(self, path):
        self.capture = cv2.VideoCapture(path)
    def read(self):
        return self.capture.read()
    def isOpened(self):
        return self.capture.isOpened()
    def release(self):
        self.capture.release()
    def getCap(self):
        return self.capture
