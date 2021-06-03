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
        self.pwm = GPIO.PWM(self.Ena, 100)
        self.pwm.start(0)

    def moveForward(self, speed):
       
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        self.pwm.ChangeDutyCycle(abs(speed))
        if speed > 0:
            GPIO.output(self.In1, GPIO.LOW)
            GPIO.output(self.In2, GPIO.HIGH)
        else:
            GPIO.output(self.In1, GPIO.HIGH)
            GPIO.output(self.In2, GPIO.LOW)
    def moveBackward(self, speed=30, angle=0):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        self.pwm.ChangeDutyCycle(abs(speed))
        if speed > 0:
            GPIO.output(self.In1, GPIO.HIGH)
            GPIO.output(self.In2, GPIO.LOW)
        else:
            GPIO.output(self.In1, GPIO.LOW)
            GPIO.output(self.In2, GPIO.HIGH)

    def stop(self):
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.In1, GPIO.LOW)
        GPIO.output(self.In2, GPIO.LOW)