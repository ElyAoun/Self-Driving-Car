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
    def stopLR(self,t=0):
        self.motor_left.stop()
        self.motor_right.stop()
        sleep(t)