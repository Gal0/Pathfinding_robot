import smbus
import time
import CalibratedComp


#rightDeg = [274.7, 274.7, 274.7]
#rightDeg = [276.0, 276.6, 278.2, 279.2, 280.2, 281.2, 282.2, 283.2, 284.2, 285.2,
           # 286.2, 287.2, 288.2, 289.2, 290.2, 291.2, 292.2, 293.2, 294.2, 295.2,
           # 296.2, 297.2, 298.2, 299.2, 300.2, 301.2, 302.2, 303.2,  304.2, 305.2, 306.2, 307.2, 308.2, 309.2, 310.2, 311.2, 312.2, 313.2, 314.2, 315.2, 316.2, 317.2, 318.2, 319.2, 320.2, 321.2, 322.2, 323.2, 324.2, 325.2, 326.2, 327.2, 328.2, 329.2, 330.2, 331.2, 332.2, 333.2, 334.2, 335.2, 336.2, 337.2, 338.2, 339.2, 340.2, 341.2, 342.2, 343.2, 344.2, 345.2, 346.2, 347.2, 348.2, 349.2, 350.2, 351.2, 352.2, 353.2, 354.2, 355.2, 356.2, 357.2, 358.2, 359.2,
           # 0.2, 1.2, 2.2, 3.2, 4.2, 5.2, 6.2, 7.2, 8.2, 9.2, 10.2, 11.2, 12.2, 13.2, 14.2, 15.2, 16.2, 17.2, 18.2, 19.2, 20.2, 21.2, 22.2, 23.2, 24.2, 25.2, 26.2, 27.2, 28.2, 29.2, 30.2, 31.2, 32.2, 33.2, 34.2, 35.2, 36.2, 37.2, 38.2, 39.2, 40.2, 41.2, 42.2, 43.2, 44.2, 45.2, 46.2, 47.2, 48.2, 49.2, 50.2, 51.2, 52.2, 53.2, 54.2, 55.2, 56.2, 57.2, 58.2, 59.2, 60.2, 61.2, 62.2, 63.2, 64.2, 65.2, 66.2, 67.2, 68.2, 69.2, 70.2, 71.2, 72.2, 73.2, 74.2, 75.2, 76.2, 77.2, 78.2, 79.2, 80.2, 81.2, 82.2, 83.2, 84.2, 85.2, 86.2, 87.2, 88.2, 89.2, 90.2, 91.2, 92.2, 93.2, 94.2, 95.2, 96.2, 97.2, 98.2, 99.2, 100.2, 101.2, 102.2, 103.2, 104.2, 105.2, 106.2, 107.2, 108.2, 109.2, 110.2, 111.2, 112.2, 113.2, 114.2, 115.2, 116.2, 117.2, 118.2, 119.2, 120.2, 121.2, 122.2, 123.2, 124.2, 125.2, 126.2, 127.2, 128.2, 129.2, 130.2, 131.2, 132.2, 133.2, 134.2, 135.2, 136.2, 137.2, 138.2, 139.2, 140.2, 141.2, 142.2, 143.2, 144.2, 145.2, 146.2, 147.2, 148.2, 149.2, 150.2, 151.2, 152.2, 153.2, 154.2, 155.2, 156.2, 157.2, 158.2, 159.2, 160.2, 161.2, 162.2, 163.2, 164.2, 165.2, 166.2, 167.2, 168.2, 169.2, 170.2, 171.2, 172.2, 173.2, 174.2, 175.2, 176.2, 177.2, 178.2, 179.2, 180.2, 181.2, 182.2, 183.2, 184.2, 185.2, 186.2, 187.2, 188.2, 189.2, 190.2, 191.2, 192.2, 193.2, 194.2, 195.2, 196.2, 197.2, 198.2, 199.2, 200.2, 201.2, 202.2, 203.2, 204.2, 205.2, 206.2, 207.2, 208.2, 209.2, 210.2, 211.2, 212.2, 213.2, 214.2, 215.2, 216.2, 217.2, 218.2, 219.2, 220.2, 221.2, 222.2, 223.2, 224.2, 225.2, 226.2, 227.2, 228.2, 229.2, 230.2, 231.2, 232.2, 233.2, 234.2, 235.2, 236.2, 237.2, 238.2, 239.2, 240.2, 241.2, 242.2, 243.2, 244.2, 245.2, 246.2, 247.2, 248.2, 249.2, 250.2, 251.2, 252.2, 253.2, 254.2, 255.2, 256.2, 257.2, 258.2, 259.2, 260.2, 261.2, 262.2, 263.2, 264.2, 265.2, 266.2, 267.2, 268.2, 269.2]
#leftDeg = [94.8]
#leftDeg = [88.6, 89.7, 90.1, 91.1, 92.1, 93.1, 94.1, 95.1, 96.1, 97.1, 98.1, 99.1, 100.1, 101.1, 102.1, 103.1, 104.1, 105.1, 106.1, 107.1, 108.1, 109.1, 110.1, 111.1, 112.1, 113.1, 114.1, 115.1, 116.1, 117.1, 118.1, 119.1, 120.1, 121.1, 122.1, 123.1, 124.1, 125.1, 126.1, 127.1, 128.1, 129.1, 130.1, 131.1, 132.1, 133.1, 134.1, 135.1, 136.1, 137.1, 138.1, 139.1, 140.1, 141.1, 142.1, 143.1, 144.1, 145.1, 146.1, 147.1, 148.1, 149.1, 150.1, 151.1, 152.1, 153.1, 154.1, 155.1, 156.1, 157.1, 158.1, 159.1, 160.1, 161.1, 162.1, 163.1, 164.1, 165.1, 166.1, 167.1, 168.1, 169.1, 170.1, 171.1, 172.1, 173.1, 174.1, 175.1, 176.1, 177.1, 178.1, 179.1, 180.1, 181.1, 182.1, 183.1, 184.1, 185.1, 186.1, 187.1, 188.1, 189.1, 190.1, 191.1, 192.1, 193.1, 194.1, 195.1, 196.1, 197.1, 198.1, 199.1, 200.1, 201.1, 202.1, 203.1, 204.1, 205.1, 206.1, 207.1, 208.1, 209.1, 210.1, 211.1, 212.1, 213.1, 214.1, 215.1, 216.1, 217.1, 218.1, 219.1, 220.1, 221.1, 222.1, 223.1, 224.1, 225.1, 226.1, 227.1, 228.1, 229.1, 230.1, 231.1, 232.1, 233.1, 234.1, 235.1, 236.1, 237.1, 238.1, 239.1, 240.1, 241.1, 242.1, 243.1, 244.1, 245.1, 246.1, 247.1, 248.1, 249.1, 250.1, 251.1, 252.1, 253.1, 254.1, 255.1, 256.1, 257.1, 258.1, 259.1, 260.1, 261.1, 262.1, 263.1, 264.1, 265.1, 266.1, 267.1, 268.1, 269.1, 270.1, 271.1, 272.1, 273.1, 274.1, 275.1, 276.1, 277.1, 278.1, 279.1, 280.1, 281.1, 282.1, 283.1, 284.1, 285.1, 286.1, 287.1, 288.1, 289.1, 290.1, 291.1, 292.1, 293.1, 294.1, 295.1, 296.1, 297.1, 298.1, 299.1, 300.1, 301.1, 302.1, 303.1, 304.1, 305.1, 306.1, 307.1, 308.1, 309.1, 310.1, 311.1, 312.1, 313.1, 314.1, 315.1, 316.1, 317.1, 318.1, 319.1, 320.1, 321.1, 322.1, 323.1, 324.1, 325.1, 326.1, 327.1, 328.1, 329.1, 330.1, 331.1, 332.1, 333.1, 334.1, 335.1, 336.1, 337.1, 338.1, 339.1, 340.1, 341.1, 342.1, 343.1, 344.1, 345.1, 346.1, 347.1, 348.1, 349.1, 350.1, 351.1, 352.1, 353.1, 354.1, 355.1, 356.1, 357.1, 358.1, 359.1,
          # 0.1, 1.1, 2.1, 3.1, 4.1, 5.1, 6.1, 7.1, 8.1, 9.1, 10.1, 11.1, 12.1, 13.1, 14.1, 15.1, 16.1, 17.1, 18.1, 19.1, 20.1, 21.1, 22.1, 23.1, 24.1, 25.1, 26.1, 27.1, 28.1, 29.1, 30.1, 31.1, 32.1, 33.1, 34.1, 35.1, 36.1, 37.1, 38.1, 39.1, 40.1, 41.1, 42.1, 43.1, 44.1, 45.1, 46.1, 47.1, 48.1, 49.1, 50.1, 51.1, 52.1, 53.1, 54.1, 55.1, 56.1, 57.1, 58.1, 59.1, 60.1, 61.1, 62.1, 63.1, 64.1, 65.1, 66.1, 67.1, 68.1, 69.1, 70.1, 71.1, 72.1, 73.1, 74.1, 75.1, 76.1, 77.1, 78.1, 79.1, 80.1, 81.1, 82.1, 83.1, 84.1, 85.1, 86.1, 87.1, 88.1, 89.1]

#rightDeg = [(y+90 for y in range(270)), (y for y in range(90))] #How many degrees in the frame of the magnetometer must be passed in order to rotate by 90° in the frame of a highly accurate compass (for left/right Rotation)

#leftDeg = [(y+270 for y in range(90)), (y for y in range(270))]

bus = smbus.SMBus(1) # set bus

DEVICE = 0x20 # Device address (A0-A2)
IODIRA = 0x00 #reigster for input-output direction of GPA
IODIRB = 0x01 #reigster for input-output direction of GPB
OLATA = 0x14 #register for output (GPA)
OLATB = 0x15 #register for output (GPB)

stepDelay = 0.0025 #0.002
compassMeasureDelay = 4 #the heading should be measured only every n'th step to decrease power consumption
correctionSteps = 25#35 #How many correctionSteps should be tried
rotTol = 1.5 #Tolerance of rotation in degrees; (+-)


bus.write_byte_data(DEVICE,IODIRA,0x00) #Define all GPA pins as output
bus.write_byte_data(DEVICE,IODIRB,0x00) #Define all GPB pins as output
bus.write_byte_data(DEVICE,OLATA,0x00) #all outputs are inactive
bus.write_byte_data(DEVICE,OLATB,0x00) #all outputs are inactive

fullstep_seq = [None for x in range(4)]

stepsPerCm = 34 #how many steps is one cm foward-motion
stepsPerRightRotation = 730 #How many steps for one right-rotation (90° rotation)
stepsPerLeftRotation = 750 #How many steps for one left-rotation (90° rotation)

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

halfstep_seq_old = [
  [1,0,0,0],
  [1,1,0,0],
  [0,1,0,0],
  [0,1,1,0],
  [0,0,1,0],
  [0,0,1,1],
  [0,0,0,1],
  [1,0,0,1]
]

def forward(stepNum):
    for i in range(stepNum): #512 = 1 revolution 1*512
      for step in (range(4)): #reversed -> backwards reversed(range(8)): 4 = halfstep, 8 = fullstep
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[2][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[2][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def backward(stepNum):
    for i in range(stepNum): #512 = 1 revolution 1*512
      for step in reversed(range(4)): #reversed -> backwards reversed(range(8)): 4 = halfstep, 8 = fullstep
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[2][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[2][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateLeft(stepNum):
    for i in range(stepNum): #512 = 1 revolution 1*512
      for step in reversed(range(4)): #reversed -> backwards reversed(range(8)): 4 = halfstep, 8 = fullstep
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[3][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[3][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateRight(stepNum):
    for i in range(stepNum): #512 = 1 revolution 1*512
      for step in (range(4)): #reversed -> backwards reversed(range(8)): 4 = halfstep, 8 = fullstep
        bus.write_byte_data(DEVICE,OLATA,fullstep_seq[3][step])
        bus.write_byte_data(DEVICE,OLATB,fullstep_seq[3][step])
        time.sleep(stepDelay)
    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)

def rotateLeftC(stepNum, currHeading): #rotation includes correction #currHeading = the heading the robot SHOULD! have currently
    rotateLeft(stepNum)
    
    #roundedHeading = round(currHeading)
    #restDegrees = currHeading-roundedHeading    
    
    toDegree = currHeading + 90###int(roundedHeading) + 90 # + restDegrees #+90° because it is a left rotation #toDregree = to which value of degree the robot schould move next
    if toDegree >= 360: toDegree -= 360
     
    rotationCorrection(toDegree)
    
    return toDegree

    
def rotateRightC(stepNum,currHeading): #rotation includes correction
    rotateRight(stepNum)
 
    roundedHeading = round(currHeading)
    restDegrees = abs(currHeading-roundedHeading)

    toDegree = currHeading - 90 ###int(roundedHeading) - 90 # + restDegrees #-90° because it is a left rotation #toDregree = to which value of degree the robot schould move next
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
    
        if heading > toDegree-rotTol and heading < toDegree+rotTol: #heading lies in range of tolerance
            bus.write_byte_data(DEVICE,OLATA,0x00)
            bus.write_byte_data(DEVICE,OLATB,0x00)
            print("IN TOLERANCE! Delta: ", abs(deltaHeading))
            print("endHeading: ", CalibratedComp.getAvHeading(3))
            return

        if abs(deltaHeading) < 180:
            if deltaHeading < 0:
                rotateLeft(compassMeasureDelay)
                #print("RL1")
            if deltaHeading > 0:
                rotateRight(compassMeasureDelay)
                #print("RR1")
        else:
            if deltaHeading < 0:
                rotateRight(compassMeasureDelay)
                #print("RR2")
            if deltaHeading > 0:
                rotateLeft(compassMeasureDelay)
                #print("RL2")

    bus.write_byte_data(DEVICE,OLATA,0x00)
    bus.write_byte_data(DEVICE,OLATB,0x00)
    print("NOT IN TOLERANCE! Delta: ", deltaHeading)
      
bus.write_byte_data(DEVICE,OLATA,0x00)
bus.write_byte_data(DEVICE,OLATB,0x00)
    ######Neue Lösung: Rotation um eine gewisse Stepnumber die etwa 90° ist und dann nachträgliches üperprüfen der Gradzahl und dann Korrektur durch erneute Rotation in Richtung der 90°
 
#rotateRight(stepsPerRightRotation) 
#print(CalibratedComp.getAvHeading())
#rotateRightC(stepsPerRightRotation, CalibratedComp.getAvHeading(12), 0, 90)
#rotateLeftC(stepsPerLeftRotation, CalibratedComp.getAvHeading(12), 0, 90)
#rotateLeft(760)
#forward(15*34)
#rotateLeft(1020)
#forward(5*34)