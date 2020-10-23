#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''
import numpy as np

from matplotlib import pyplot as plt
import matplotlib.patches as patches

import cozmo

from frame2d import Frame2D


class Coord2D:
    def __init__(self, xp: float, yp: float):
        self.x = xp
        self.y = yp
    def __str__(self):
        return "[x="+str(self.x)+",y="+str(self.y)+"]"

class Coord2DGrid:
    def __init__(self, xp: int, yp: int):
        self.x = xp
        self.y = yp
    def __str__(self):
        return "[index-x="+str(self.x)+",index-y="+str(self.y)+"]"

class OccupancyGrid:
    FREE = 0
    OCCUPIED = 1
    def __init__(self, start: Coord2D, stepSize, sizeX, sizeY):
        self.gridStart = start
        self.gridStepSize = stepSize
        self.gridSizeX = sizeX
        self.gridSizeY = sizeY
        self.gridData = np.zeros((sizeX,sizeY),int)

    def validateIndex(self, c: Coord2D):
        if c.x < 0 or self.gridSizeX <= c.x:
            raise Exception("OccupancyGrid coordinate ", str(c), " is out of bounds.")
        if c.y < 0 or self.gridSizeY <= c.y:
            raise Exception("OccupancyGrid coordinate ", str(c), " is out of bounds.")

    def validateIndexStop(self, c: Coord2D):
        if c.x < -1 or self.gridSizeX < c.x:
            raise Exception("OccupancyGrid coordinate ", str(c), " is out of bounds for index stop.")
        if c.y < -1 or self.gridSizeY < c.y:
            raise Exception("OccupancyGrid coordinate ", str(c), " is out of bounds for index stop.")
    
    def float2grid(self, c: Coord2D):
        xIndex = round( (c.x - self.gridStart.x) / self.gridStepSize )
        yIndex = round( (c.y - self.gridStart.y) / self.gridStepSize )
        ci = Coord2DGrid(xIndex,yIndex)
        self.validateIndex(ci)
        return ci

    def grid2float(self, ci: Coord2DGrid):
        self.validateIndex(ci)
        x = self.gridStart.x + ci.x*self.gridStepSize
        y = self.gridStart.y + ci.y*self.gridStepSize
        return Coord2D(x,y)

    def isFreeGrid(self, ci: Coord2DGrid):
        self.validateIndex(ci)
        return self.gridData[int(ci.x),int(ci.y)] == self.FREE

    def isFree(self, c: Coord2D):
        return self.isFreeGrid(self.float2grid(c))

    def isOccupiedGrid(self, ci: Coord2DGrid):
        self.validateIndex(ci)
        return self.gridData[int(ci.x),int(ci.y)] == self.OCCUPIED

    def isOccupied(self, c: Coord2D):
        return self.isOccupiedGrid(self.float2grid(c))

    def setFree(self, start: Coord2DGrid, end: Coord2DGrid):
        self.validateIndex(start)
        self.validateIndexStop(end)
        for x in range(start.x, end.x):
            for y in range(start.y, end.y):
                self.gridData[x,y] = self.FREE

    def setOccupied(self, start: Coord2DGrid, end: Coord2DGrid):
        self.validateIndex(start)
        self.validateIndexStop(end)
        for x in range(start.x, end.x):
            for y in range(start.y, end.y):
                self.gridData[x,y] = self.OCCUPIED
        
    def minX(self):
        return self.gridStart.x - 0.5*self.gridStepSize

    def minY(self):
        return self.gridStart.y - 0.5*self.gridStepSize

    def maxX(self):
        return self.gridStart.x + (self.gridSizeX - 0.5)*self.gridStepSize

    def maxY(self):
        return self.gridStart.y + (self.gridSizeY - 0.5)*self.gridStepSize

    def __str__(self):
        g = ""
        for x in range(0, self.gridSizeX):
            line = ""
            for y in range(0, self.gridSizeY):
                if self.gridData[x,y] == self.FREE:
                    line = line+".. "
                elif self.gridData[x,y] == self.OCCUPIED:
                    line = line+"XX "
            g = g+line+"\n"
        return g


# Map storing an occupancy grip and a set of landmarks
# langmarks are stored in a dictionary mapping  cube-IDs on Frame2D (indicating their true position)
class CozmoMap:
	def __init__(self, grid, landmarks, targets=None):
		self.grid = grid
		self.landmarks = landmarks
		self.targets = targets
	



def loadU08520Map():
	# based on a 60cm x 80cm layout
	sizeX = 32
	#sizeY = 42
	sizeY = 44
	grid = OccupancyGrid(Coord2D(-10,-10), 20.0, sizeX, sizeY)
	grid.setOccupied(Coord2DGrid(0,0), Coord2DGrid(sizeX,sizeY))
	grid.setFree(Coord2DGrid(1,1), Coord2DGrid(sizeX-1,sizeY-1))
	grid.setOccupied(Coord2DGrid(16,21), Coord2DGrid(sizeX,23))

	landmarks = {
		cozmo.objects.LightCube1Id : Frame2D.fromXYA(540,40,0),
		cozmo.objects.LightCube2Id : Frame2D.fromXYA(40,440,0),
		cozmo.objects.LightCube3Id : Frame2D.fromXYA(540,500,0) }
	
	targets = [Frame2D.fromXYA(400,760,0)]
	
	return CozmoMap(grid,landmarks,targets)


def plotMap(ax, m : CozmoMap, color="blue"):
	grid = m.grid
	minX = grid.minX()
	maxX = grid.maxX()
	minY = grid.minY()
	maxY = grid.maxY()
	tick = grid.gridStepSize
	numX = grid.gridSizeX
	numY = grid.gridSizeY
	for xIndex in range (0,numX+1):
		x = minX + xIndex*tick
		bold = 0.8 if (xIndex-1)%5==0 else 0.4
		plt.plot([x, x], [minY,maxY], color, alpha=bold, linewidth=bold)
	for yIndex in range (0,numY+1):
		y = minY + yIndex*tick
		bold = 0.8 if (yIndex-1)%5==0 else 0.4
		plt.plot([minX,maxX], [y, y], color, alpha=bold, linewidth=bold)

	for xIndex in range (0,numX):
		for yIndex in range (0,numY):
			if grid.isOccupiedGrid(Coord2DGrid(xIndex,yIndex)):
				rect = patches.Rectangle((minX+tick*xIndex,minY+tick*yIndex),tick,tick,linewidth=1,edgecolor='blue',facecolor='blue',zorder=0)
				ax.add_patch(rect)

	for landmark in m.landmarks:
		x = m.landmarks[landmark].x()
		y = m.landmarks[landmark].y()
		a = m.landmarks[landmark].angle()
		size = 22
		rect = patches.Rectangle((x-size,y-size),2*size,2*size,linewidth=2,edgecolor='blue',facecolor="none",zorder=0)
		ax.add_patch(rect)

	for target in m.targets:
		x = target.x()
		y = target.y()
		size = 20
		ell = patches.Ellipse((x,y),2*size,2*size,linewidth=2,edgecolor='blue',facecolor=[0,0,1,0.5],zorder=0)
		ax.add_patch(ell)
		ell2 = patches.Ellipse((x,y),4*size,4*size,linewidth=2,linestyle=":",edgecolor='blue',facecolor="none",zorder=0)
		ax.add_patch(ell2)
		


	
def testMapVis():
	m=loadU08520Map()
	print(m.grid)

	fig = plt.figure(figsize=(8, 8))
	ax = fig.add_subplot(1, 1, 1, aspect=1)
	ax.set_xlim(m.grid.minX(), m.grid.maxX())
	ax.set_ylim(m.grid.minY(), m.grid.maxY())


	plotMap(ax,m)
	plt.show()



