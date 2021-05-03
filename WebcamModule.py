import cv2

cap = cv2.VideoCapture(0)


def getImg(display=False, size=(480, 240)):
    _, img = cap.read()
    img = cv2.resize(img, (size[0], size[1]))
    if display:
        cv2.imshow('IMG', img)
    return img

class Webcam():
    def __init__(self,captureNumber,screenWidth=320,screenHeight=240):
        self.capture=cv2.VideoCapture(captureNumber)
        self.capture.set(3,screenWidth)
        self.capture.set(4,screenHeight)



if __name__ == '__main__':
    while True:
        img = getImg(True)
