import RPi.GPIO as GPIO
from MotorModule import Motor
from time import sleep

class CarModule:
    def __init__(self, motor_left, motor_right):
        self.motor_left = motor_left
        self.motor_right = motor_right

    def moveLRForward(self, speed=50, angle=0, t=0):
        self.motor_left.moveForward(speed - angle)
        self.motor_right.moveForward(speed + angle)
        sleep(t)
        self.stopLR()

    def moveLRBackward(self, speed=50, angle=0, t=0):
        self.motor_left.moveBackward(speed - angle)
        self.motor_right.moveBackward(speed + angle)
        sleep(t)
        self.stopLR()

    def stopLR(self, t=0):
        self.motor_left.stop()
        self.motor_right.stop()
        sleep(t)
