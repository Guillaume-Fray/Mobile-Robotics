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
position = initial_pose

# this probability distribution represents a uniform distribution over the entire map in any orientation
mapPrior = Uniform(
		np.array([m.grid.minX(), m.grid.minY(), 0]),
		np.array([m.grid.maxX(), m.grid.maxY(), 2*math.pi]))


#
numParticles = 200

# The main data structure: array for particles, each represnted as Frame2D
particles = sampleFromPrior(mapPrior, numParticles)

#noise injected in re-sampling process to avoid multiple exact duplications of a particle
#
xyaResampleVar = np.diag([10, 30, 5*math.pi/180])  # x: 5-10, y:30, theta: 5

# note here: instead of creating new gaussian random numbers every time, which is /very/ expensive,
# 	precompute a large table of them and recycle. GaussianTable does that internally
xyaResampleNoise = GaussianTable(np.zeros([3]), xyaResampleVar, 10000)  # 10000

# Motor error model
cozmoOdomNoiseX = 0.01
cozmoOdomNoiseY = 0.01
cozmoOdomNoiseTheta = 0.000006
xyaNoiseVar = np.diag([cozmoOdomNoiseX, cozmoOdomNoiseY, cozmoOdomNoiseTheta])
xyaNoise = GaussianTable(np.zeros([3]), xyaNoiseVar, 10000)



def runMCLLoop(simWorld: CozmoSimWorld, finished):
	global particles
	global position
	
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
		print(' ')

		# MCL step 1: prediction (shift particle through motion model), basically add x,y and theta noise
		# to model's predictions
		delta = track_speed_to_pose_change(lspeed, rspeed, interval).toXYA()
		visible = False
		# For each particle
		for i in range(0, numParticles):
			next_x_w_noise = delta[0] + cozmoOdomNoiseX
			next_y_w_noise = delta[1] + cozmoOdomNoiseY
			next_a_w_noise = delta[2] + cozmoOdomNoiseTheta
			delta_with_noise = Frame2D.fromXYA(next_x_w_noise, next_y_w_noise, next_a_w_noise)

			currentParticles[i] = currentParticles[i].mult(delta_with_noise)
		# See run-odom-vis.py under "Run simulations" 

		# MCL step 2: weighting (weigh particles with sensor model)
		for i in range(0, numParticles):
			cliff_weight = cozmo_cliff_sensor_model(currentParticles[i], m, cliffDetected)
			inv_curr_particles = currentParticles[i].inverse()

			for cube in cubeIDs:
				rel_real_cube_pose = inv_curr_particles.mult(simWorld.cube_pose_global(cube))
				rel_measured_cube_pose = cubeRelativeFrames[cube]
				cube_sensor_weight = cube_sensor_model(rel_real_cube_pose, cubeVisibility[cube], rel_measured_cube_pose)
				particleWeights[i] = cliff_weight*cube_sensor_weight
				if cubeVisibility[cube]:
					visible = True

		# MCL step 3: resampling (proportional to weights)
		# newParticles = resampleIndependent(currentParticles, particleWeights, numParticles , xyaResampleNoise)
		if visible or cliffDetected:
			num_new_samples = 3  # or 5
		else:
			num_new_samples = 0

		fresh_samples = sampleFromPrior(mapPrior, num_new_samples)
		newParticles = resampleLowVar(currentParticles, particleWeights, numParticles - num_new_samples, xyaResampleNoise)
		# For instance: under which circumstances could it be better to insert no fresh samples at all?

		# write global variable
		particles = newParticles + fresh_samples

		print("t = "+str(t), end="\r\n")
		t = t+1

		t1 = time.time()
		timeTaken = t1-t0
		if timeTaken < interval:
			time.sleep(interval - timeTaken)
		else:
			print("Warning: loop iteration tool more than "+str(interval) + " seconds (t="+str(timeTaken)+")", end="\r\n")


def average_particles():
	global position
	x = 0
	y = 0
	a = 0

	for i in range(0, numParticles):
		x = x + particles[i]
		y = y + particles[i]
		a = a + particles[i]

	x = x/numParticles
	y = y/numParticles
	a = a/numParticles

	position = Frame2D.fromXYA(x, y, a)

	return position


def explore():
	simWorld.drive_wheel_motors(40, 40)
	time.sleep(5)
	simWorld.drive_wheel_motors(30, -30)
	time.sleep(3)




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
		for i in range(0, numParticles):
			particlesXYA[i, :] = particles[i].toXYA()

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
	cubeIDs = [cozmo.objects.LightCube1Id, cozmo.objects.LightCube2Id, cozmo.objects.LightCube3Id]
	cliff_detected = simWorld.is_cliff_detected()
	visible = False
	global position

	while not finished.is_set():
		# TODO --- this part was used for testing the MC implementation and parameters settings (TASK 3)
		simWorld.drive_wheel_motors(30, -30)
		time.sleep(3.5)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(15)
		simWorld.drive_wheel_motors(-30, 30)
		time.sleep(3)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(6)
		simWorld.drive_wheel_motors(-30, 30)
		time.sleep(2.7)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(8)
		simWorld.drive_wheel_motors(30, 14)
		time.sleep(18)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(7)
		simWorld.drive_wheel_motors(-40, 40)
		time.sleep(4)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(12)
		simWorld.drive_wheel_motors(40, -40)
		time.sleep(3)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(13)
		simWorld.drive_wheel_motors(40, -40)
		time.sleep(3)
		simWorld.drive_wheel_motors(30, 30)
		time.sleep(5)
		finished.set()
		# TODO --- End of testing









		# TODO --- This part below is not working  ---
		# explore()

		# position = average_particles()

		# inv_current_pose = position.inverse()
		# relative_target = inv_current_pose.mult(target)

		# rel_targ = relative_target.toXYA()
		# x = rel_targ[0]
		# y = rel_targ[1]
		# d = math.sqrt(x * x + y * y)  # distance between current position and target position

		# if not (d < 50):
			# explore()

			# if d < 50:
				# print("Target found!")
				# finished.set()

			# drive away from the cliff guard
			# if cliff_detected:
				# simWorld.drive_wheel_motors(-40, 40)
				# time.sleep(3)
				# simWorld.drive_wheel_motors(30, 30)
				# time.sleep(3)


		# velocity = target_pose_to_velocity_spline(relative_target)
		# track_speed = velocity_to_track_speed(velocity[0], velocity[1])
		# lspeed = simWorld.left_wheel_speed()
		# rspeed = simWorld.right_wheel_speed()
		# delta = track_speed_to_pose_change(lspeed, rspeed, interval)
		# position = position.mult(delta)

		# simWorld.drive_wheel_motors(track_speed[0], track_speed[1])
		# time.sleep(interval)


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

		

