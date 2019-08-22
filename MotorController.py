#Date: 07.08.2019
#Author: Robin Drescher with help from https://tutorials-raspberrypi.de/gpios-mittels-i2c-port-expander-erweitern/ 
#for the implementation of the port expander.

import smbus
import time
import CalibratedComp

bus = smbus.SMBus(1) #set bus

DEVICE = 0x20 #device address (A0-A2)
IODIRA = 0x00 #reigster for input-output direction of GPA
IODIRB = 0x01 #reigster for input-output direction of GPB
OLATA = 0x14 #register for output (GPA)
OLATB = 0x15 #register for output (GPB)

stepDelay = 0.0025 #how much time there is between two steps
compassMeasureDelay = 4 #the heading should be measured only every n'th step to decrease power consumption
correctionSteps = 25#35 #How many correctionSteps should be tried
rotTol = 1.5 #tolerance of rotation in degrees; (+-)

bus.write_byte_data(DEVICE,IODIRA,0x00) #Define all GPA pins as output
bus.write_byte_data(DEVICE,IODIRB,0x00) #Define all GPB pins as output
bus.write_byte_data(DEVICE,OLATA,0x00) #all outputs are inactive
bus.write_byte_data(DEVICE,OLATB,0x00) #all outputs are inactive

fullstep_seq = [None for x in range(4)]

stepsPerCm = 34 #how many steps is one cm foward-motion
stepsPerRightRotation = 730 #How many steps needed for one right-rotation (90° rotation)
stepsPerLeftRotation = 750 #How many steps needed for one left-rotation (90° rotation)

fullstep_seq[0] = [ #GPA=Backwheels OR GPB=frontwheels, seq for only left wheel
  0x09,
  0x0C,
  0x06,
  0x03
]
fullstep_seq[1] = [ #seq for only right wheel
  0x10,
  0x20,
  0x40,
  0x80
]
fullstep_seq[2] = [ #seq for both left AND right wheel - SAME DIRECTION
  0x19,
  0x2C,
  0x46,
  0x83
]
fullstep_seq[3] = [ #seq for both left AND right wheel - OPPOSITE DIRECTION
  0x99,
  0xCC,
  0x66,
  0x33
]

halfstep_seq = [None for x in range(4)] 

halfstep_seq[0] = [ #GPA=Backwheels OR GPB=frontwheels, seq for only left wheel
  0x09,
  0x08,
  0x0C,
  0x04,
  0x06,
  0x02,
  0x03,
  0x01
]
halfstep_seq[1] = [ #seq for only right wheel
  0x10,
  0x30,
  0x20,
  0x60,
  0x40,
  0xC0,
  0x80,
  0x90
]
halfstep_seq[2] = [ #seq for both left AND right wheel - SAME DIRECTION
  0x19,
  0x38,
  0x2C,
  0x64,
  0x46,
  0xC2,
  0x83,
  0x91
]
halfstep_seq[3] = [ #seq for both left AND right wheel - OPPOSITE DIRECTION
  0x99,
  0x88,
  0xCC,
  0x44,
  0x66,
  0x22,
  0x33,
  0x11,
]

def forward(stepNum):
    for i in range(stepNum):
      for step in (range(4)): #range(4) = halfstep(faster) | range(8) = fullstep(more precision)
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[2][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[2][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def backward(stepNum):
    for i in range(stepNum):
      for step in reversed(range(4)): #reversed -> backwards motion of wheels
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[2][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[2][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateLeft(stepNum):
    for i in range(stepNum):
      for step in reversed(range(4)):
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[3][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[3][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateRight(stepNum):
    for i in range(stepNum):
      for step in (range(4)):
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[3][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[3][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateLeftC(stepNum, currHeading): #rotation includes correction. #currHeading = the heading the robot should have currently have
    rotateLeft(stepNum)
    
    
    toDegree = currHeading + 90 #+90° because it is a left rotation #toDregree = to which value of degree the robot schould move next
    if toDegree >= 360: toDegree -= 360    
    rotationCorrection(toDegree)  
    return toDegree

    
def rotateRightC(stepNum,currHeading): #rotation includes correction
    rotateRight(stepNum)
 
    roundedHeading = round(currHeading)
    restDegrees = abs(currHeading-roundedHeading)

    toDegree = currHeading - 90 #-90° because it is a right rotation
    if toDegree < 0: toDegree += 360
    
    
    rotationCorrection(toDegree)
    
    return toDegree
    
def rotationCorrection(toDegree):

    for i in range(correctionSteps): #512 = 1 revolution 1*512
        
        heading = CalibratedComp.getAvHeading(3)      
        deltaHeading = heading - toDegree
        
        
        print("toDegree: ", toDegree)
        print("heading: ", heading)
        print("deltaheading: ", deltaHeading)
    
        if heading > toDegree-rotTol and heading < toDegree+rotTol: #check if heading lies in range of tolerance, if yes, stop correction
            bus.write_byte_data(DEVICE,OLATA,0x00)
            bus.write_byte_data(DEVICE,OLATB,0x00)
            print("IN TOLERANCE! Delta: ", abs(deltaHeading))
            print("endHeading: ", CalibratedComp.getAvHeading(3))
            return

        if abs(deltaHeading) < 180:
            if deltaHeading < 0:
                rotateLeft(compassMeasureDelay)
            if deltaHeading > 0:
                rotateRight(compassMeasureDelay)
        else:
            if deltaHeading < 0:
                rotateRight(compassMeasureDelay)
            if deltaHeading > 0:
                rotateLeft(compassMeasureDelay)

    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)
    print("NOT IN TOLERANCE! Delta: ", deltaHeading)
      
bus.write_byte_data(DEVICE,OLATA,0x00)
bus.write_byte_data(DEVICE,OLATB,0x00)
