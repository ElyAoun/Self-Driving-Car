import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


class Motor:
    def __init__(self, Ena, In1, In2):
        self.Ena = Ena
        self.In1 = In1
        self.In2 = In2
        GPIO.setup(self.Ena, GPIO.OUT)
        GPIO.setup(self.In1, GPIO.OUT)
        GPIO.setup(self.In2, GPIO.OUT)
        self.pwmA = GPIO.PWM(self.Ena, 100)
        self.pwmA.start(0)

    def moveForward(self, speed=50, angle=0):
        self.pwmA.ChangeDutyCycle(speed)
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100
        if speed > 0:
            GPIO.output(self.In1, GPIO.HIGH)
            GPIO.output(self.In2, GPIO.LOW)
        else:
            GPIO.output(self.In1, GPIO.LOW)
            GPIO.output(self.In2, GPIO.HIGH)

    def stop(self, t=0):
        GPIO.output(self.In1, GPIO.LOW)
        GPIO.output(self.In2, GPIO.LOW)
        sleep(t)
