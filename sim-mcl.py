#!/usr/bin/env python3


import asyncio
import cozmo
from cozmo_interface import *
from cozmo_sim_world_plot import *
from terminal_reader import WaitForChar
from gaussian import Gaussian, GaussianTable, plotGaussian
import math
import numpy as np
import threading
import time


# this data structure represents the map
m = loadU08520Map()
initial_pose = Frame2D.fromXYA(100, 200, 3.1416/2)
current_pose = initial_pose

# this probability distribution represents a uniform distribution over the entire map in any orientation
mapPrior = Uniform(
		np.array([m.grid.minX(), m.grid.minY(), 0]),
		np.array([m.grid.maxX(), m.grid.maxY(), 2*math.pi]))


# TODO Major parameter to choose: number of particles
numParticles = 50

# The main data structure: array for particles, each represnted as Frame2D
particles = sampleFromPrior(mapPrior, numParticles)

#noise injected in re-sampling process to avoid multiple exact duplications of a particle
# TODO Choose sensible re-sampling variation
xyaResampleVar = np.diag([10, 10, 10*math.pi/180])

# note here: instead of creating new gaussian random numbers every time, which is /very/ expensive,
# 	precompute a large table of them and recycle. GaussianTable does that internally
xyaResampleNoise = GaussianTable(np.zeros([3]), xyaResampleVar, 10000)

# Motor error model
cozmoOdomNoiseX = 0.01
cozmoOdomNoiseY = 0.01
cozmoOdomNoiseTheta = 0.000006
xyaNoiseVar = np.diag([cozmoOdomNoiseX, cozmoOdomNoiseY, cozmoOdomNoiseTheta])
xyaNoise = GaussianTable(np.zeros([3]), xyaNoiseVar, 10000)



def runMCLLoop(simWorld: CozmoSimWorld, finished):
	global particles
	global current_pose
	
	particleWeights = np.zeros([numParticles])
	cubeIDs = [cozmo.objects.LightCube1Id, cozmo.objects.LightCube2Id, cozmo.objects.LightCube3Id]

	# main loop
	interval = 0.25
	t = 0
	while not finished.is_set():
		t0 = time.time()
		
		# read cube sensors
		cubeVisibility = {}
		cubeRelativeFrames = {}
		numVisibleCubes = 0
		for cubeID in cubeIDs:
			relativePose = Frame2D()
			visible = False
			if simWorld.cube_is_visible(cubeID):
				relativePose = simWorld.cube_pose_relative(cubeID)
				visible = True
				numVisibleCubes = numVisibleCubes+1
			cubeVisibility[cubeID] = visible
			cubeRelativeFrames[cubeID] = relativePose

		# read cliff sensor
		cliffDetected = simWorld.is_cliff_detected()

		# read track speeds
		lspeed = simWorld.left_wheel_speed()
		rspeed = simWorld.right_wheel_speed()

		# read global variable
		currentParticles = particles

		# MCL step 1: prediction (shift particle through motion model), basically add x,y and theta delta_with_noise
		# to model's predictions
		delta = track_speed_to_pose_change(lspeed, rspeed, interval).toXYA()
		# For each particle
		for i in range(0, numParticles):
			# TODO
			next_x_w_noise = delta[0] + cozmoOdomNoiseX
			next_y_w_noise = delta[1] + cozmoOdomNoiseY
			next_a_w_noise = delta[2] + cozmoOdomNoiseTheta
			delta_with_noise = Frame2D.fromXYA(next_x_w_noise, next_y_w_noise, next_a_w_noise)

			# dx = 200-currentParticles[i].x()
			# dy = 350-currentParticles[i].y()
			# v = math.sqrt(lspeed*lspeed + rspeed*rspeed)

			currentParticles[i] = currentParticles[i].mult(delta_with_noise)
			# TODO Instead: shift particles along deterministic motion model, then add perturbation with xyaNoise (see above)
		# See run-odom-vis.py under "Run simulations" 

		# MCL step 2: weighting (weigh particles with sensor model)
		# 	cube_sensor_model(true_cube_position, visible, measured_position
		for i in range(0, numParticles):
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

		print("t = "+str(t), end="\r\n")
		t = t+1

		t1 = time.time()
		timeTaken = t1-t0
		if timeTaken < interval:
			time.sleep(interval - timeTaken)
		else:
			print("Warning: loop iteration tool more than "+str(interval) + " seconds (t="+str(timeTaken)+")", end="\r\n")
			


def runMCLPlotLoop(m: CozmoMap, simWorld: CozmoSimWorld, finished):
	global particles

	# create plot
	plt.ion()
	plt.show()
	fig = plt.figure(figsize=(8, 8))
	ax = fig.add_subplot(1, 1, 1, aspect=1)

	ax.set_xlim(m.grid.minX(), m.grid.maxX())
	ax.set_ylim(m.grid.minY(), m.grid.maxY())

	plotMap(ax, m)

	particlesXYA = np.zeros([numParticles, 3])
	for i in range(0, numParticles):
		particlesXYA[i, :] = particles[i].toXYA()
	particlePlot = plt.scatter(particlesXYA[:, 0], particlesXYA[:, 1], color="red", zorder=3, s=10, alpha=0.5)

	empiricalG = Gaussian.fromData(particlesXYA[:, 0:2])
	gaussianPlot = plotGaussian(empiricalG, color="red")
	
	robotPlot = plotRobot(simWorld.dont_touch__pos())
	cube1Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube1Id))
	cube2Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube2Id))
	cube3Plot = plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube3Id))
 
	# main loop
	t = 0
	while not finished.is_set():
		# update plot	
		for i in range(0,numParticles):
			particlesXYA[i,:] = particles[i].toXYA()
		particlePlot.set_offsets(particlesXYA[:, 0:2])

		empiricalG = Gaussian.fromData(particlesXYA[:, 0:2])
		plotGaussian(empiricalG, color="red", existingPlot=gaussianPlot)

		plotRobot(simWorld.dont_touch__pos(), existingPlot=robotPlot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube1Id), existingPlot=cube1Plot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube2Id), existingPlot=cube2Plot)
		plotLandmark(simWorld.cube_pose_global(cozmo.objects.LightCube3Id), existingPlot=cube3Plot)

		plt.draw()
		plt.pause(0.001)
		
		time.sleep(0.01)





def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):

	on_target = False
	target = Frame2D.fromXYA(400, 760, 90*(math.pi / 180))

	while not finished.is_set():
		# simWorld.drive_wheel_motors(60, -60)
		# time.sleep(10)
		# simWorld.drive_wheel_motors(30, 30)
		# time.sleep(20)
		# simWorld.drive_wheel_motors(30, 21)
		# time.sleep(10)

		simWorld.drive_wheel_motors(20, 8)
		time.sleep(53)

		# if cube 1 visible:
			# turn until you see cube 2
			# go towards cube 2 until you know where you are
			# then go towards target

		# if cube 2 visible:
			# turn until you see cube 3
			# go towards cube 3 until you know where you are
			# then go towards target

		# if cube 3 visible:
			# turn until you see cube 2
			# go towards cube 3 until you know where you are
			# then go towards target



		finished.set()


interval = 0.1



def cozmo_program(simWorld: CozmoSimWorld):
	finished = threading.Event()
	print("Starting simulation. Press Q to exit", end="\r\n")
	threading.Thread(target=runWorld, args=(simWorld,finished)).start()
	threading.Thread(target=runMCLLoop, args=(simWorld,finished)).start()
	threading.Thread(target=WaitForChar, args=(finished,'[Qq]')).start()
	threading.Thread(target=runCozmoMainLoop, args=(simWorld,finished)).start()
	# running the plot loop in a thread is not thread-safe because matplotlib
	# uses tkinter, which in turn has a threading quirk that makes it
	# non-thread-safe outside the python main program.
	# See https://stackoverflow.com/questions/14694408/runtimeerror-main-thread-is-not-in-main-loop
    
	# threading.Thread(target=runPlotLoop, args=(simWorld,finished)).start()
	runMCLPlotLoop(m, simWorld, finished)


# NOTE: this code allows to specify the initial position of Cozmo on the map
simWorld = CozmoSimWorld(m, initial_pose)

cozmo_program(simWorld)

		

