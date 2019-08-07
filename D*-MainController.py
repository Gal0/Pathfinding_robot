#Date: 27.07.2019
#D*-Algorithm by Howie Choset (with variations)

from tkinter import *
import math
import time
import DistSensor
import MotorController

#---CONSTANTS---
#canvas dimensions
pixelPerCm = 1.2
blockedTileCost = 99999
#grid properties
tileSize= 15#in cm
xTiles, yTiles = 21, 21 #number of tiles in x/y direction
xStart, yStart = 10, 15 #grid-position of start-tile
xGoal, yGoal = 10,12#10, 13 #grid-position of goal-tile
startTile = None ##From where the robot starts his pathfinding
#Canvas colors
freeTileColor = "#B0DFE5"
obsticleColor = "red"
pathColor = "yellow"

cWidth = xTiles*tileSize*pixelPerCm
cHeight = yTiles*tileSize*pixelPerCm
tilePixelSize = tileSize*pixelPerCm #size of one tile in pixel

tiles = [[None for y in range(yTiles)] for x in range(xTiles)] 

class Tile:
    x, y = None, None #x/y grid-position; starting from 0
    neighborTiles = []
    obsticle = False
    tag = 0 #0=NEW, 1=OPEN, 2=CLOSED
    cost = None #cost from starttile to this tile
    prevTile = None #from which tile did the path come from?
    key = None #D* smallest value of cost(X) since X was placed on open list
    
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
    orientation = 0 # 0=north, 1=east, 2=south, 3=west
    isMoving = False
    path = []
    def __init__(self, x, y):
        self.x = x
        self.y = y
                               
                
                
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬CANVAS    
def SetUpCanvas():
    #__Set up Canvas
    global root
    root = Tk()
    #root.geometry("500x400")
    global canvas
    canvas = Canvas(root, width=cWidth, height=cHeight)
    canvas.pack()  

def VisualizeTiles():
    for j in range(yTiles):
        for i in range(xTiles):
            color = freeTileColor
            if(tiles[i][j].obsticle == True):
                color = obsticleColor
            canvas.create_rectangle(i*tilePixelSize, j*tilePixelSize, (i+1)*tilePixelSize, (j+1)*tilePixelSize, fill=color)

def VisualizePath(robot):        
    count = 0
    for tile in robot.path: #shows pathlines by converting grid-positions to positions on the canvas
        canvas.create_line(gridToCanvas(robot.path[count].x), gridToCanvas(robot.path[count].y), gridToCanvas(robot.path[count+1].x), gridToCanvas(robot.path[count+1].y), width = 2, fill = pathColor)
        count+=1
        if count >= len(robot.path)-1:
            break
        
def UpdateCanvas():
    Canvas.delete("all")
    VisualizeTiles()
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
def Insert(tile, newCost):
  if newCost > blockedTileCost:
      newCost = blockedTileCost
      
  if tile.tag == 0: #New
      tile.cost = newCost
      tile.key = newCost
      openTiles.append(tile)
      tile.tag = 1

  if tile.tag == 1: #Open
      tile.cost = newCost
      if newCost < tile.key:
          tile.key = newCost

  if tile.tag == 2: #Closed
      tile.cost = newCost
      if newCost < tile.key:
          tile.key = newCost 
      tile.tag = 1
      openTiles.append(tile)

def PathBlocked(robot, blockedTile): 
    Insert(blockedTile, blockedTile.cost) #the blocked tile is added to the openList ...
    for neigh in blockedTile.neighborTiles: #... and also its neighbors
        Insert(neigh, neigh.cost)
        print(robot.x,robot.y, "|||", blockedTile.x, blockedTile.y)
    DStarSearch(robot)
    
def DStarSearch(robot):
    robot.path = []
    startTile = tiles[robot.x][robot.y]
    

    if startTile == goalTile:
       return

    while len(openTiles) != 0:
        currentTile = GetLowPriorityTile(openTiles)
        if currentTile == tiles[xStart][yStart]:
            break
        

        if currentTile.key < currentTile.cost: #If it is a raise-state
            for neigh in currentTile.neighborTiles:
                if neigh.cost < currentTile.key and currentTile.cost > neigh.cost + PathDistance(currentTile, neigh):
                    currentTile.prevTile = neigh
                    currentTile.cost = neigh.cost + PathDistance(currentTile, neigh)

        if currentTile.key == currentTile.cost:
            for neigh in currentTile.neighborTiles: 

                if neigh.tag == 0 or (neigh.prevTile == currentTile and neigh.cost != currentTile.cost + PathDistance(currentTile, neigh)) or (neigh.prevTile != currentTile and neigh.cost > currentTile.cost + PathDistance(currentTile, neigh)):

               
                    newCost = currentTile.cost + PathDistance(currentTile, neigh)
                    Insert(neigh, newCost)
                    neigh.prevTile = currentTile
        else:
            for neigh in currentTile.neighborTiles:
                if neigh.tag == 0 or (neigh.prevTile == currentTile and neigh.cost != currentTile.cost + PathDistance(currentTile, neigh)):
                    neigh.prevTile = currentTile
                    Insert(neigh, currentTile.cost + PathDistance(currentTile, neigh))

                elif neigh.prevTile != currentTile and neigh.cost > currentTile.cost + PathDistance(currentTile, neigh):
                    Insert(currentTile, currentTile.cost)

                elif neigh.prevTile != currentTile and currentTile.cost > neigh.cost + PathDistance(currentTile, neigh) and neigh.tag == 2 and neigh.cost > currentTile.key:
                    Insert(neigh, neigh.cost)
          
        openTiles.remove(currentTile)
        currentTile.tag = 2 #current Tile gets closed
        

    #go from goal backwards to start with the prevTiles(startTile included), reverse it 
    robot.path.append(startTile)
    currentPrevTile = startTile.prevTile
    while True:
        robot.path.append(currentPrevTile)
        if currentPrevTile == goalTile:
            break
        currentPrevTile = currentPrevTile.prevTile
            
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
    
        print(dist, gridDist, "Num of Sens", i)
    
        if gridDist == 0:
            print("robot is too close to an obsticle")
            gridDist = 1
            #continue
    
        trajectory = robot.orientation
    
        if i == 0: #if it's the distSensor on the left side
            trajectory = (trajectory-1) % 4
        
        if i == 3: #if it's the distSensor on the right side
            trajectory = (trajectory+1) % 4
                   
        
        if trajectory == 0: #if DistSensor is facing north            
            if robot.y - gridDist < 0:
                print("obsticle out of tilemap")
                updateFreeSpace(robot.x, robot.y, robot.x, 0)
                continue
            tiles[robot.x][robot.y-gridDist].obsticle = True
            updateFreeSpace(robot.x, robot.y, robot.x, robot.y-gridDist+1)
            
        if trajectory == 1: #if DistSensor is facing east
            if robot.x + gridDist > xTiles-1:
                print("obsticle out of tilemap")
                updateFreeSpace(robot.x, robot.y, xTiles-1, robot.y)
                continue
            tiles[robot.x+gridDist][robot.y].obsticle = True
            updateFreeSpace(robot.x, robot.y, robot.x+gridDist-1, robot.y)
            
        if trajectory == 2: #if DistSensor is facing south
            if robot.y + gridDist > yTiles-1:
                print("obsticle out of tilemap")
                updateFreeSpace(robot.x, robot.y, robot.x, yTiles-1)
                continue
            tiles[robot.x][robot.y+gridDist].obsticle = True
            updateFreeSpace(robot.x, robot.y, robot.x, robot.y+gridDist-1)
            
        if trajectory == 3: #if DistSensor is facing west
            if robot.x - gridDist < 0:
                print("obsticle out of tilemap")
                updateFreeSpace(robot.x, robot.y, 0, robot.y)
                continue
            tiles[robot.x-gridDist][robot.y].obsticle = True
            updateFreeSpace(robot.x, robot.y, robot.x-gridDist+1, robot.y)
        
def updateFreeSpace(x1,y1,x2,y2): #If the robot detected an obsticle earlier which is no longer there, the obsticale bool gets reset to false for this tile  
    if y1 > y2: #north
        y1+=-1
        while y1 >= y2:
            tiles[x1][y1].obsticle = False
            y1+=-1
        return
    
    if x1 < x2: #east
        x1+=1
        while x1 <= x2:
            tiles[x1][y1].obsticle = False
            x1+=1
        return

    if y1 < y2: #south
        y1+=1
        while y1 <= y2:
            tiles[x1][y1].obsticle = False
            y1+=1
        return
        
    if x1 > x2: #west
        x1+=-1
        while x1 >= x2:
            tiles[x1][y1].obsticle = False
            x1+=-1
        return
        
    return
#¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬¬Observation


          
    

def Move(robot):

    nextTile = robot.path[1]

    if nextTile.obsticle == True:
        PathBlocked(robot, nextTile)
        return
    
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
        if robot.orientation == 0:
            robot.y += -1
        if robot.orientation == 1:
            robot.x += 1
        if robot.orientation == 2:
            robot.y += 1
        if robot.orientation == 3:
            robot.x += -1
        
    if tileDir == 1: #if true, rotate right
        MotorController.rotateRight(MotorController.stepsPerRightRotation)
        robot.orientation = (robot.orientation+1) % 4
        
    if tileDir == 2: #if true, rotate 180°
        MotorController.rotateRight(MotorController.stepsPerRightRotation*2)
        robot.orientation = (robot.orientation+2) % 4
        
    if tileDir == 3: #if true, rotate left
        MotorController.rotateLeft(MotorController.stepsPerLeftRotation)
        robot.orientation = (robot.orientation-1) % 4
        
    robot.isMoving = False
    return
    
    
    
   
def Refresher():
    print("REF")
    Observe(robot)      

    UpdateCanvas()
    
     if(len(robot.path) > 1 and robot.isMoving == False):
        Move(robot)


    if len(robot.path) < 2:
        UpdateCanvas()
        return

    Refresher()




#-----------------MAIN-----------------
          
#__Set up tiles
for j in range(yTiles):
    for i in range(xTiles):
        tiles[i][j] = Tile(i, j)

oalTile = tiles[xGoal][yGoal]

openTiles.append(goalTile)  
goalTile.tag = 1
goalTile.cost = 0
goalTile.key = 0    
        
#__Find each tile's neighbors
for j in range(yTiles):
    for i in range(xTiles):
        tiles[i][j].SetNeighborTiles()

#__Setp up Robot
robot = Robot (xStart, yStart)       

SetUpCanvas()

DStarSearch(robot)
Refresher()
root.mainloop()

