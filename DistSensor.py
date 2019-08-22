#Date: 07.08.2019
#Author: https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/

import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

sonicSpeed = 34300
triggerInterval = 0.00001

distSensorAmount = 4
triggerPins = [16, 20, 12, 21] #Sensor order: left, middle-left, middle-right, right
echoPins = [13, 19, 6, 26]

distSensors = [None for i in range(distSensorAmount)]

class DistSens:
    GPIO_TRIG = None
    GPIO_ECHO = None
    distance = None #distance measured by this distance sensor
        
    def GetDistance(self):
        self.CalcDistance()
        return self.distance
    
    def CalcDistance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIG, True)
 
        # set Trigger after 0.01ms to LOW
        time.sleep(triggerInterval)
        GPIO.output(self.GPIO_TRIG, False)
 
        StartTime = time.time()
        StopTime = time.time()
 
        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()
 
        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            StopTime = time.time()
 
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * sonicSpeed) / 2
 
        self.distance = distance
        
    def Set(self):
        #set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIG, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)
        
    
    def __init__(self, trig, echo):
        self.GPIO_TRIG = trig
        self.GPIO_ECHO = echo
        self.Set()
