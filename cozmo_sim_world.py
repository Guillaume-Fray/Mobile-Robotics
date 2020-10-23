
import cozmo

from frame2d import Frame2D 

from map import CozmoMap, Coord2D

from mcl_tools import *

import math
import numpy as np
import random
import time


class CozmoSimWorld:
	def __init__(self, m, pos):
		self.map = m
		self.__dont_touch__pos = pos
		self.__dont_touch__s1 = 0
		self.__dont_touch__s2 = 0
		self.__dont_touch__s1Command = 0
		self.__dont_touch__s2Command = 0
		self.__dont_touch__cliff_sensed = False
		self.__dont_touch__cube_visibility = {
			cozmo.objects.LightCube1Id : False,
			cozmo.objects.LightCube2Id : False,
			cozmo.objects.LightCube3Id : False }
		self.waitTime = 0.05
		self.gotStuck = False

	def dont_touch__pos(self):
		return self.__dont_touch__pos
	
	def dont_touch__step(self):
		if not self.gotStuck:
			x = self.__dont_touch__pos.x()
			y = self.__dont_touch__pos.y()
			if not is_in_map(self.map, x, y):
				self.gotStuck = True
			elif self.map.grid.isOccupied(Coord2D(x,y)):
				self.gotStuck = True
		
		s1 = self.__dont_touch__s1*self.waitTime
		s2 = self.__dont_touch__s2*self.waitTime
		a = (s2-s1)/(np.exp(4.44265125649))
		if abs(a) > 0.0001:
			rad = (s2+s1)/(2*a)
			x = rad*math.sin(a)
			y = -rad*(math.cos(a)-1)
		else:
			x = s1
			y = 0
		d1 = self.__dont_touch__s1 - self.__dont_touch__s1Command
		d2 = self.__dont_touch__s2 - self.__dont_touch__s2Command
		accel = d1*d1+d2*d2
		# multiplicative noise on x and a, scaled by acceleration, to simulate slippage
		x = x + x*np.random.normal(0,min(0.5,0.0001*accel))
		a = a + a*np.random.normal(0,min(0.5,0.0001*accel))
		f = Frame2D.fromXYA(x,y,a)
		if not self.gotStuck:
			self.__dont_touch__pos = self.__dont_touch__pos.mult(f)

		self.__dont_touch__cliff_sensed = self.__dont_touch__compute_is_cliff_detected()

		for cubeID in self.__dont_touch__cube_visibility:
			self.__dont_touch__cube_visibility[cubeID] = self.__dont_touch__compute_cube_is_visible(cubeID)

		self.__dont_touch__s1 = 0.9*self.__dont_touch__s1+0.1*self.__dont_touch__s1Command
		self.__dont_touch__s2 = 0.9*self.__dont_touch__s2+0.1*self.__dont_touch__s2Command


	def __dont_touch__compute_cube_is_visible(self, cubeID):
		relativePose = self.__dont_touch__pos.inverse().mult(self.map.landmarks[cubeID])
		angle = math.atan2(relativePose.y(),relativePose.x())
		midAngle = 30.0/180.0*math.pi
		relativeAngle = abs(angle)/midAngle
		relativeTolerance = 0.1
		if 1+relativeTolerance < relativeAngle:
			angleVisibilityProb = 0.0
		elif 1-relativeTolerance < relativeAngle:
			angleVisibilityProb = 0.5
		else:
			angleVisibilityProb = 0.99

		distance = math.sqrt(relativePose.y()*relativePose.y()+relativePose.x()*relativePose.x())
		minDistance = 100
		minTolerance = 50
		maxDistance = 600
		maxTolerance = 100
		if distance < minDistance - minTolerance:
			distanceProb = 0.0
		elif distance < minDistance + minTolerance:
			distanceProb = 0.99 * (distance-(minDistance-minTolerance))/(2*minTolerance)
		elif distance < maxDistance - maxTolerance:
			distanceProb = 0.99
		elif distance < maxDistance + maxTolerance:
			distanceProb = 0.99 - 0.99 * (distance-(maxDistance-maxTolerance))/(2*maxTolerance)
		else:
			distanceProb = 0.0
		
		p = angleVisibilityProb*distanceProb

		if random.random()<p:
			return True
		return False

	def __dont_touch__compute_is_cliff_detected(self):
		p = cozmo_cliff_sensor_model(self.__dont_touch__pos, self.map, True)
		if random.random()<p:
			return True
		return False

	def _touch_only_for_experiments_get_position(self):
		return self.__dont_touch__pos

	def _touch_only_for_experiments_set_position(self, newPose):
		self.__dont_touch__pos = newPose

	def cube_is_visible(self, cubeID):
		return self.__dont_touch__cube_visibility[cubeID]

	def is_cliff_detected(self):
		return self.__dont_touch__cliff_sensed

	def drive_wheel_motors(self,l_wheel_speed,r_wheel_speed):
		self.__dont_touch__s1Command = max(-100,min(l_wheel_speed,100))
		self.__dont_touch__s2Command = max(-100,min(r_wheel_speed,100))

	def cube_pose_relative(self, cubeID):
		if not self.cube_is_visible(cubeID):
			return Frame2D()

		relativePose = self.__dont_touch__pos.inverse().mult(self.map.landmarks[cubeID])

		distance = math.sqrt(relativePose.y()*relativePose.y()+relativePose.x()*relativePose.x())

		dx = np.random.normal(0,5) + np.random.normal(0,5)*(distance/200)
		dy = np.random.normal(0,3) + np.random.normal(0,3)*(distance/200)
		da = np.random.normal(0,0.05)

		sensedRelative = Frame2D.fromXYA(  relativePose.x()+dx, relativePose.y()+dy, relativePose.angle()+da  )

		return sensedRelative

	def cube_pose_global(self, cubeID):
		if not self.cube_is_visible(cubeID):
			return Frame2D()

		return self.__dont_touch__pos.mult(self.cube_pose_relative(cubeID))

	def left_wheel_speed(self):
		if random.random() < 0.05:
			return 0
		return float(int(self.__dont_touch__s1))

	def right_wheel_speed(self):
		if random.random() < 0.05:
			return 0
		return float(int(self.__dont_touch__s2))
		

def runWorld(w:CozmoSimWorld,finished):
	while not finished.is_set():
		t0 = time.time()
		w.dont_touch__step()
		t1 = time.time()
		timeTaken = t1-t0
		if timeTaken < w.waitTime:
			time.sleep(w.waitTime - timeTaken)




