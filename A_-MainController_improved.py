from tkinter import *
import math
import time
import pickle
import DistSensor
import MotorController
import CalibratedComp

#---CONSTANTS---
#canvas dimensions
#cWidth, cHeight = 900, 900
pixelPerCm = 1.7
#grid properties
tileSize= 15#in cm
xTiles, yTiles = 21, 21 #number of tiles in x/y direction
xStart, yStart = 10, 15 #grid-position of start-tile
xGoal, yGoal = 10,8#10, 13 #grid-position of goal-tile
#Canvas colors
freeTileColor = "#C8B5A6"
robotColor = "#F0F1FC"
goalTileColor = "#29BC17"
obsticleColor = "red"
pathColor = "yellow"

#pixelPerCm = cWidth/(tileSize*(xTiles))
cWidth = xTiles*tileSize*pixelPerCm
cHeight = yTiles*tileSize*pixelPerCm
tilePixelSize = tileSize*pixelPerCm #size of one tile in pixel

tiles = [[None for y in range(yTiles)] for x in range(xTiles)]
loadPrevMap = False #If True, the previous stored map will be loaded.

deactUpdateFreeSpace = True #If this bool is set to true , the rbot won't remove an obsticle from his grid map, when he doesn't see it anymore
mapUpdated = 0 #If Sensors find that an absticle has moved away, this  variable keeps track of it, so that the pathfinding is re-done #0: not updated, 1:updated necessary, 2: update done

class Tile:
    x, y = None, None #x/y grid-position; starting from 0
    neighborTiles = []
    obsticle = False
    cost = None #cost from starttile to this tile
    prevTile = None #from which tile did the path come from?
    priority = None #A* prioritizes tiles closer to the goal

    
    def SetNeighborTiles(self):
        #horizontal/vertical
        if(self.x != xTiles-1): #right
            self.neighborTiles.append(tiles[self.x+1][self.y])
        if(self.x != 0): #left
            self.neighborTiles.append(tiles[self.x-1][self.y])
        if(self.y != 0): #up
            self.neighborTiles.append(tiles[self.x][self.y-1])
        if(self.y != yTiles-1): #down
            self.neighborTiles.append(tiles[self.x][self.y+1])
#        #diagonal
#        if(self.x != xTiles-1 and self.y != 0): #right-up
#            self.neighborTiles.append(tiles[self.x+1][self.y-1])
#        if(self.x != 0 and self.y != 0): #left-up
#            self.neighborTiles.append(tiles[self.x-1][self.y-1])
#        if(self.x != xTiles-1 and self.y != yTiles-1): #right-down
#            self.neighborTiles.append(tiles[self.x+1][self.y+1])
#        if(self.x != 0 and self.y != yTiles-1): #left-down
#            self.neighborTiles.append(tiles[self.x-1][self.y+1])
    
    def __init__(self,x, y):
        self.x = x
        self.y = y
        self.neighborTiles = []

class Robot:
    x, y = None, None #position in grid-coordinate-system
    currentHeading = None
    orientation = 0 # 0=north, 1=east, 2=south, 3=west
    isMoving = False
    path = []
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.currentHeading = CalibratedComp.getAvHeading(12)
        print(self.currentHeading)
                
                
                
                
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬CANVAS    
def SetUpCanvas():
    #__Set up Canvas
    global root
    root = Tk()
    #root.geometry("500x400")
    global canvas
    canvas = Canvas(root, width=cWidth, height=cHeight)
    canvas.pack()  

def VisualizeTiles(robot):
    for j in range(yTiles):
        for i in range(xTiles):
            color = freeTileColor
            
            if(tiles[i][j].obsticle == True):
                color = obsticleColor      
            if(i == xGoal and j == yGoal):
                color = goalTileColor     
            if(i == robot.x and j == robot.y):
                color = robotColor
                
            canvas.create_rectangle(i*tilePixelSize, j*tilePixelSize, (i+1)*tilePixelSize, (j+1)*tilePixelSize, fill=color)

def VisualizePath(robot):
   for i in range(len(robot.path)):
        if i >= len(robot.path)-1:
            break
        canvas.create_line(gridToCanvas(robot.path[i].x), gridToCanvas(robot.path[i].y), gridToCanvas(robot.path[i+1].x), gridToCanvas(robot.path[i+1].y), width = 3, fill = pathColor)
        
def UpdateCanvas(robot):
    canvas.delete("all")
    VisualizeTiles(robot)
    VisualizePath(robot)
    root.update()
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬CANVAS
    
          

def gridToCanvas(x): #converts a position on the grid-coordinate-system to the canvas
    return tilePixelSize*(x+0.5)
    
def gridToCm(x): #converts a position in the grid-coordinate-system to cm-coordinate-system
    return tilePixelSize*(x+0.5)/pixelPerCm

def PathDistance(fromTile, toTile): #Calculates the cost to move from one tile to another
    if toTile.obsticle == True:
        return 99999
    return ((fromTile.x-toTile.x)**2 + (fromTile.y-toTile.y)**2)**0.5 #magnitude of length between the two tiles
    
    return _cost

def GetLowPriorityTile(openTiles): #returns the tile in openTiles with the least value of priority which will be prioritized
    leastPriTile = None #Tile with least prioirty
    for tile in openTiles:
        if leastPriTile == None or tile.priority < leastPriTile.priority:
            leastPriTile = tile
        
    return leastPriTile

#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬Pathfinding
def AStarSearch(robot):
    robot.path = []
    startTile = tiles[robot.x][robot.y]
    openTiles = [] #these tiles will be checked next
    currentTile = None #current tile which's neighbors will be checked
    openTiles.append(startTile)
    startTile.cost = 0
    startTile.priority = 0
    while len(openTiles) != 0:
        currentTile = GetLowPriorityTile(openTiles)
        if currentTile == goalTile:
            break
        
        for neigh in currentTile.neighborTiles:
            #calculate priority
            newCost = currentTile.cost + PathDistance(currentTile, neigh)
            if neigh.cost == None or newCost < neigh.cost:
                neigh.cost = newCost
                neigh.priority = newCost + PathDistance(neigh, goalTile)
                neigh.prevTile = currentTile
                openTiles.append(neigh)
            
        openTiles.remove(currentTile)

    #go from goal backwards to start with the prevTiles(startTile included), reverse it
    robot.path.append(goalTile)
    currentPrevTile = goalTile.prevTile
    while True:
        robot.path.append(currentPrevTile)
        if currentPrevTile == startTile:
            break
        currentPrevTile = currentPrevTile.prevTile
    robot.path.reverse()

def ResetPathfinding():
    for j in range(yTiles):
        for i in range(xTiles):
            tiles[i][j].cost = None
            #tiles[i][j].obsticle = False #Robot forgets all obsticles which isn't so good!!!!!!
            
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬Pathfinding
   
   
   
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬Observation

def Observe(robot): #receives sensordata and converts this information
    for i in range(DistSensor.distSensorAmount):
        dist = DistSensor.distSensors[i].GetDistance()
        
#        if dist > 200: #measurement maximal 200cm
#            print("too far away")
#            continue

        #assuming robot is positioned in the middle of a tile
        gridDist = math.floor(dist/tileSize) #rounded number of tiles the obsticle is away
    
        #print(dist, gridDist, "Num of Sens", i)
    
        if gridDist == 0:
            #print("robot is too close to an obsticle")
            gridDist = 1
            #continue
    
        trajectory = robot.orientation
    
        if i == 0: #if it's the distSensor on the left side
            trajectory = (trajectory-1) % 4
        
        if i == 3: #if it's the distSensor on the right side
            trajectory = (trajectory+1) % 4

        
        if trajectory == 0: #if DistSensor is facing north            
            if robot.y - gridDist < 0:
                #print("obsticle out of tilemap")
                UpdateFreeSpace(robot.x, robot.y, robot.x, 0)
                continue
            tiles[robot.x][robot.y-gridDist].obsticle = True
            UpdateFreeSpace(robot.x, robot.y, robot.x, robot.y-gridDist+1)
            
        if trajectory == 1: #if DistSensor is facing east
            if robot.x + gridDist > xTiles-1:
                #print("obsticle out of tilemap")
                UpdateFreeSpace(robot.x, robot.y, xTiles-1, robot.y)
                continue
            tiles[robot.x+gridDist][robot.y].obsticle = True
            UpdateFreeSpace(robot.x, robot.y, robot.x+gridDist-1, robot.y)
            
        if trajectory == 2: #if DistSensor is facing south
            if robot.y + gridDist > yTiles-1:
                #print("obsticle out of tilemap")
                UpdateFreeSpace(robot.x, robot.y, robot.x, yTiles-1)
                continue
            tiles[robot.x][robot.y+gridDist].obsticle = True
            UpdateFreeSpace(robot.x, robot.y, robot.x, robot.y+gridDist-1)
            
        if trajectory == 3: #if DistSensor is facing west
            if robot.x - gridDist < 0:
                #print("obsticle out of tilemap")
                UpdateFreeSpace(robot.x, robot.y, 0, robot.y)
                continue
            tiles[robot.x-gridDist][robot.y].obsticle = True
            UpdateFreeSpace(robot.x, robot.y, robot.x-gridDist+1, robot.y)
        
def UpdateObsitcle(x1,y1):
      if  tiles[x1][y1].obsticle == True:        
            tiles[x1][y1].obsticle = False
            global mapUpdated
            if mapUpdated == 0 : mapUpdated = 1   

def UpdateFreeSpace(x1,y1,x2,y2): #If the robot detected an obsticle earlier which is no longer there, the obsticale bool gets reset to false for this tile
    
    if deactUpdateFreeSpace == True:
        return
    
    if y1 > y2: #north
        y1+=-1
        while y1 >= y2:
            UpdateObsitcle(x1,y1)              
            y1+=-1       
        return
    
    if x1 < x2: #east
        x1+=1
        while x1 <= x2:
            UpdateObsitcle(x1,y1)
            x1+=1
        return

    if y1 < y2: #south
        y1+=1
        while y1 <= y2:
            UpdateObsitcle(x1,y1)
            y1+=1
        return
        
    if x1 > x2: #west
        x1+=-1
        while x1 >= x2:
            UpdateObsitcle(x1,y1)
            x1+=-1
        return
        
    return
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬Observation


          
    

def Move(robot):
    nextTile = robot.path[1]

    
    if nextTile.obsticle == True:
        ResetPathfinding()
        AStarSearch(robot)
        return
    
    global mapUpdated
    if mapUpdated == 1:
        ResetPathfinding()
        AStarSearch(robot)
        mapUpdated = 2
        return

    mapUpdated = 0

    robot.isMoving = True

    
    tileDir = None #in which direction nextTile is in the frame of the robot, 0=north, 1=east, 2=south, 3=west
    if nextTile.y < robot.y:
        tileDir = 0
    if nextTile.x > robot.x:
        tileDir = 1
    if nextTile.y > robot.y:
        tileDir = 2
    if nextTile.x < robot.x:
        tileDir = 3
        
    tileDir = (tileDir - robot.orientation) % 4
          
        
    if tileDir == 0: #if true, move forward
        MotorController.forward(MotorController.stepsPerCm * tileSize)
        del robot.path[0]
        if robot.orientation == 0:
            robot.y += -1
        if robot.orientation == 1:
            robot.x += 1
        if robot.orientation == 2:
            robot.y += 1
        if robot.orientation == 3:
            robot.x += -1       
        
    if tileDir == 1: #if true, rotate right
        robot.currentHeading = MotorController.rotateRightC(MotorController.stepsPerRightRotation, robot.currentHeading)
        robot.orientation = (robot.orientation+1) % 4
        
    if tileDir == 2: #if true, rotate 180°
        robot.currentHeading = MotorController.rotateRightC(MotorController.stepsPerRightRotation, robot.currentHeading) #Two right rotations = Uturn
        robot.currentHeading = MotorController.rotateRightC(MotorController.stepsPerRightRotation, robot.currentHeading)
        robot.orientation = (robot.orientation+2) % 4
        
    if tileDir == 3: #if true, rotate left
        robot.currentHeading = MotorController.rotateLeftC(MotorController.stepsPerLeftRotation, robot.currentHeading)
        robot.orientation = (robot.orientation-1) % 4
     
    robot.currentHeading = CalibratedComp.getAvHeading() #Update heading because it can change from from to place 
    robot.isMoving = False
    return
    
    
    
   
def Refresher():
    print("REF")
    Observe(robot)      

    UpdateCanvas(robot)
    
    if(len(robot.path) > 1 and robot.isMoving == False):
        Move(robot)


    if len(robot.path) < 2:
        UpdateCanvas(robot)
        
        #save map
        print("save")
        f = open('gridMap.p', 'wb')
        pickle.dump(tiles, f)
        f.close()
        return

    Refresher()



#-----------------MAIN-----------------
       
sys.setrecursionlimit(3000)
#__Set up tiles
if loadPrevMap == False:
    for j in range(yTiles):
        for i in range(xTiles):
            tiles[i][j] = Tile(i, j)
else: #load previous gridMap
    f = open('gridMap.p', 'rb')
    tiles = pickle.load(f)
    f.close

startTile_ = tiles[xStart][yStart]
goalTile = tiles[xGoal][yGoal]
        
#__Find each tile's neighbors
for j in range(yTiles):
    for i in range(xTiles):
        tiles[i][j].SetNeighborTiles()

#__Setup up Robot
robot = Robot (xStart, yStart)

SetUpCanvas()

AStarSearch(robot)
Refresher()
root.mainloop()


