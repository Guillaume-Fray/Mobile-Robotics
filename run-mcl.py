#!/usr/bin/env python3


import asyncio

import cozmo

from frame2d import Frame2D 
from map import CozmoMap, plotMap, loadU08520Map, Coord2D
from matplotlib import pyplot as plt
from cozmo_interface import cube_sensor_model
from cozmo_interface import track_speed_to_pose_change,cozmoOdomNoiseX,cozmoOdomNoiseY,cozmoOdomNoiseTheta
from mcl_tools import *
from gaussian import Gaussian, GaussianTable, plotGaussian
from cozmo.util import degrees, distance_mm, speed_mmps
import math
import numpy as np
import threading
import time

	

# this data structure represents the map
m=loadU08520Map()

# this probability distribution represents a uniform distribution over the entire map in any orientation
mapPrior = Uniform(
		np.array([m.grid.minX(),m.grid.minY(),0]),
		np.array([m.grid.maxX(),m.grid.maxY(),2*math.pi]))


# TODO Major parameter to choose: number of particles
numParticles = 30

# The main data structure: array for particles, each represnted as Frame2D
particles = sampleFromPrior(mapPrior,numParticles)

#noise injected in re-sampling process to avoid multiple exact duplications of a particle
# TODO Choose sensible re-sampling variation
xyaResampleVar = np.diag([10,10,10*math.pi/180])
# note here: instead of creating new gaussian random numbers every time, which is /very/ expensive,
# 	precompute a large table of them an recycle. GaussianTable does that internally
xyaResampleNoise = GaussianTable(np.zeros([3]),xyaResampleVar, 10000)

# Motor error model
xyaNoiseVar = np.diag([cozmoOdomNoiseX,cozmoOdomNoiseY,cozmoOdomNoiseTheta])
xyaNoise = GaussianTable(np.zeros([3]),xyaNoiseVar,10000)


def runMCLLoop(robot: cozmo.robot.Robot):
	global particles
	
	particleWeights = np.zeros([numParticles])
	cubeIDs = [cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id]

	# main loop
	timeInterval = 0.1
	t = 0
	while True:
		t0 = time.time()
		
		# read cube sensors
		robotPose = Frame2D.fromPose(robot.pose)
		cubeVisibility = {}
		cubeRelativeFrames = {}
		numVisibleCubes = 0
		for cubeID in cubeIDs:
			cube = robot.world.get_light_cube(cubeID)
			relativePose = Frame2D()
			visible = False
			if cube is not None and cube.is_visible:
				cubePose = Frame2D.fromPose(cube.pose)
				relativePose = robotPose.inverse().mult(cubePose)
				visible = True
				numVisibleCubes = numVisibleCubes+1
			cubeVisibility[cubeID] =  visible
			cubeRelativeFrames[cubeID] =  relativePose

		# read cliff sensor
		cliffDetected = robot.is_cliff_detected

		# read track speeds
		lspeed = robot.left_wheel_speed.speed_mmps
		rspeed = robot.right_wheel_speed.speed_mmps

		# read global variable
		currentParticles = particles

		# MCL step 1: prediction (shift particle through motion model)
		# For each particle
		for i in range(0,numParticles):
			# TODO this is all wrong...
			dx = 200-currentParticles[i].x()
			dy = 350-currentParticles[i].y()
			v = math.sqrt(lspeed*lspeed + rspeed*rspeed)
			currentParticles[i] = currentParticles[i].mult(Frame2D.fromXYA(0.1*dy,-0.1*dx,0))
			# TODO Instead: shift particles along deterministic motion model, then add perturbation with xyaNoise (see above)
		# See run-odom-vis.py under "Run simulations" 

		# MCL step 2: weighting (weigh particles with sensor model)
		for i in range(0,numParticles):
			# TODO this is all wrong (again) ... 
			particleWeights[i] = cozmo_cliff_sensor_model(currentParticles[i], m, cliffDetected)
			# TODO instead, assign the product of all individual sensor models as weight (including cozmo_cliff_sensor_model!)
		# See run-sensor-model.py under "compute position beliefs"

		# MCL step 3: resampling (proportional to weights)
		# TODO not completely wrong, but not yet solving the problem
		newParticles = resampleIndependent(currentParticles, particleWeights, numParticles, xyaResampleNoise)
		# TODO Draw a number of "fresh" samples from all over the map and add them in order 
		# 		to recover form mistakes (use sampleFromPrior from mcl_tools.py)
		# TODO Keep the overall number of samples at numParticles
		# TODO Compare the independent re-sampling with "resampleLowVar" from mcl_tools.py
		# TODO Find reasonable amplitues for the resampling noise xyaResampleNoise (see above)
		# TODO Can you dynamically determine a reasonable number of "fresh" samples.
		# 		For instance: under which circumstances could it be better to insert no fresh samples at all?
		
		# write global variable
		particles = newParticles

		print("t = "+str(t))
		t = t+1

		t1 = time.time()
		timeTaken = t1-t0
		if timeTaken < timeInterval:
			time.sleep(timeInterval - timeTaken)
		else:
			print("Warning: loop iteration tool more than "+str(timeInterval) + " seconds (t="+str(timeTaken)+")")

def runPlotLoop(robot: cozmo.robot.Robot):
	global particles

	# create plot
	plt.ion()
	plt.show()
	fig = plt.figure(figsize=(8, 8))
	ax = fig.add_subplot(1, 1, 1, aspect=1)

	ax.set_xlim(m.grid.minX(), m.grid.maxX())
	ax.set_ylim(m.grid.minY(), m.grid.maxY())

	plotMap(ax,m)

	particlesXYA = np.zeros([numParticles,3])
	for i in range(0,numParticles):
		particlesXYA[i,:] = particles[i].toXYA()
	particlePlot = plt.scatter(particlesXYA[:,0],particlesXYA[:,1], color="red",zorder=3,s=10, alpha=0.5)

	empiricalG = Gaussian.fromData(particlesXYA[:,0:2])
	gaussianPlot = plotGaussian(empiricalG, color="red")

	# main loop
	t = 0
	while True:
		# update plot	
		for i in range(0,numParticles):
			particlesXYA[i,:] = particles[i].toXYA()
		particlePlot.set_offsets(particlesXYA[:,0:2])

		empiricalG = Gaussian.fromData(particlesXYA[:,0:2])
		plotGaussian(empiricalG, color="red", existingPlot=gaussianPlot)

		plt.draw()
		plt.pause(0.001)
		
		time.sleep(0.01)


def cozmo_program(robot: cozmo.robot.Robot):
	threading.Thread(target=runMCLLoop, args=(robot,)).start()
	threading.Thread(target=runPlotLoop, args=(robot,)).start()

	robot.enable_stop_on_cliff(True)

	# main loop
	# TODO insert driving and navigation behavior HERE
	t = 0
	while True:
		print("Straight")
		robot.drive_straight(distance_mm(150), speed_mmps(50)).wait_for_completed()
		print("Turn")
		robot.turn_in_place(degrees(90),speed=degrees(20)).wait_for_completed()
		print("Sleep")
		time.sleep(1)



cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(cozmo_program, use_3d_viewer=False, use_viewer=False)



		

