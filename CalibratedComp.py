#Date: 07.08.2019
#Author: Bas Huizer (https://bitbucket.org/RoboBasics/robocar/src/32836c36a411f6b33d32af49b258785220bd3ead/Handy%20stuff/HMC5883L_calibrate_step_4.py?at=master)
# (with variations)

import RPi.GPIO as GPIO
import smbus
import time
import math
import csv
import numpy

errorAtempts = 25 #How many times it is tried to re-measure the heading if an error occured
errorDelay = 0.02 #how long it is waited for the magnetometer to recover from it's malfunction

measureDelay = 0.08

rev = GPIO.RPI_REVISION
if rev == 2 or rev == 3:
    bus = smbus.SMBus(1)
else:
    bus = smbus.SMBus(0)

address = 0x1e

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)
    

def getHeading():
    write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
    write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
    write_byte(2, 0b00000000) # Continuous sampling

    scale = 0.92
    x_offset = 77.4 #Values found by calibration
    y_offset = 142
    
    x_out = (read_word_2c(3) - x_offset) * scale
    y_out = (read_word_2c(7) - y_offset) * scale
    z_out = read_word_2c(5) * scale
    
    bearing  = math.atan2(y_out, x_out) 
    if (bearing < 0):
        bearing += 2 * math.pi
        
    declination = -2.77
    bearing = math.degrees(bearing) + declination
    
    if bearing > 360:
       bearing -= 360
    
    return bearing

def getAvHeading(amount): #doesn't improve the results significantly!
    measurements = [None for a in range(amount)]
    measurements[0] = getHeading()
    
    for i in range(amount-1):
        time.sleep(measureDelay)
        for j in range(errorAtempts):
            measurements[i+1] = getHeading()
            if measurements[i+1] != measurements[i]:  #try to correct measuring error, detect it if the measurements are exactly the same, which is not expectable
                break
            time.sleep(errorDelay)
            
    return numpy.mean(measurements)
  
  
def measure():  
  for i in range(320):
     print(getHeading())
     time.sleep(0.6)
