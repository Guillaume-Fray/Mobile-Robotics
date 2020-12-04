#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''


import numpy as np
from cozmo_sim_world_plot import *
from matplotlib import pyplot as plt
from gaussian import Gaussian, plotGaussian
from cozmo_interface import *
import time
import threading
from terminal_reader import WaitForChar

np.random.seed(seed=1)


# Cozmo drives in a 30 cm wide square
def create_square_track_speeds():
	tracks_speeds = []
	for x in range(0, 150):
		tracks_speeds.append([20, 20])
	for x in range(0, 95):
		tracks_speeds.append([7, -7])

	for x in range(0, 150):
		tracks_speeds.append([20, 20])
	for x in range(0, 95):
		tracks_speeds.append([7, -7])

	for x in range(0, 150):
		tracks_speeds.append([20, 20])
	for x in range(0, 95):
		tracks_speeds.append([7, -7])

	for x in range(0, 150):
		tracks_speeds.append([20, 20])
	for x in range(0, 95):
		tracks_speeds.append([7, -7])

	return np.array(tracks_speeds)


# Cozmo drives 50 cm forward
def create_straight_track_speeds():
	tracks_speeds = []
	for x in range(0, 255):
		tracks_speeds.append([20, 20])
	for x in range(0, 260):
		tracks_speeds.append([-20, -20])

	return np.array(tracks_speeds)


# Cozmo spins on the spot
def create_spin_track_speeds():
	tracks_speeds = []
	for x in range(0, 300):
		tracks_speeds.append([-60, 60])

	return np.array(tracks_speeds)


# Cozmo spins on the spot
def create_spin_and_move_track_speeds():
	tracks_speeds = []
	for x in range(0, 400):
		tracks_speeds.append([-60, 60])
	for x in range(0, 50):
		tracks_speeds.append([30, 30])

	return np.array(tracks_speeds)


# Cozmo spins, moves forward, spins again, moves forward, turns left, moves forward ans spins.
def create_complex_track_speeds():
	tracks_speeds = []
	for x in range(0, 90):
		tracks_speeds.append([-30, 30])
	for x in range(0, 100):
		tracks_speeds.append([20, 20])

	for x in range(0, 65):
		tracks_speeds.append([-30, 30])
	for x in range(0, 100):
		tracks_speeds.append([20, 20])

	for x in range(0, 60):
		tracks_speeds.append([-8, 16])
	for x in range(0, 85):
		tracks_speeds.append([20, 20])

	for x in range(0, 150):
		tracks_speeds.append([-60, 60])

	return np.array(tracks_speeds)


# - 1 -
# track_speeds = create_square_track_speeds()
# track_speeds = create_straight_track_speeds()
# track_speeds = create_spin_track_speeds()
track_speeds = create_spin_and_move_track_speeds()
# track_speeds = create_complex_track_speeds()


# - 2 -
# Save and load file
# np.save("track_speed_square10", track_speeds)
# np.save("track_speed_straight10", track_speeds)
# np.save("track_speed_spin10", track_speeds)
# np.save("track_speed_spin_and_move30", track_speeds)
# np.save("track_speed_complex40", track_speeds)
np.save("tests_to_adjust", track_speeds)

# Setup Noise model
zero3 = np.zeros([3])
zero33 = np.zeros([3, 3])
xyaNoiseVar = np.diag([cozmoOdomNoiseX, cozmoOdomNoiseY, cozmoOdomNoiseTheta])
xyaNoise = Gaussian(zero3, xyaNoiseVar)

numParticles = 100
# 3D array for positions of particles. Index 1: Particle index. Index 2: timestep. Index 3: x,y,a
poseVecs = np.zeros([numParticles, len(track_speeds) + 1, 3])
# 2D array for frames of particles. Index 1: Particle index. Index 2: timestep.
poseFrames = []
for run in range(0, numParticles):
	poseFrames.append([Frame2D()])

# Run simulations
for t in range(0, len(track_speeds)):
	# Deterministic model
	poseChange = track_speed_to_pose_change(track_speeds[t, 0], track_speeds[t, 1], 0.1)
	poseChangeXYA = poseChange.toXYA()
	# Individual probabilistic sampling per particle
	for run in range(0, numParticles):
		# add gaussian error to x,y,a
		poseChangeXYAnoise = np.add(poseChangeXYA, xyaNoise.sample(1))
		# integrate change
		poseChangeNoise = Frame2D.fromXYA(poseChangeXYAnoise)
		pose = poseFrames[run][t].mult(poseChangeNoise)
		poseXYA = pose.toXYA()

		# keep data for further integration and visualization
		poseVecs[run, t+1, :] = poseXYA
		poseFrames[run].append(pose)

# plot each particles trajectory
for run in range(0, numParticles):
	plt.plot(poseVecs[run, :, 0], poseVecs[run, :, 1], color="blue", alpha=0.5)

# plot empirical Gaussian every 50 steps in time
for t in range(-1, len(track_speeds) - 1, 50):
	print(t)
	print(np.mean(poseVecs[:, t, :]))
	empiricalG = Gaussian.fromData(poseVecs[:, t, :])
	print(empiricalG)
	plotGaussian(empiricalG, color="red")

# scatter plot final position points
plt.scatter(poseVecs[:, len(track_speeds) - 1, 0], poseVecs[:, len(track_speeds) - 1, 1], color="red", zorder=3, s=10, alpha=0.5)


# - 3 -
# Straight, Complex
# plt.xlim(-100, 600)
# plt.ylim(-200, 200)

# Square, Spin
# plt.xlim(-200, 450)
# plt.ylim(-450, 150)

# Spin-Move
plt.xlim(-250, 250)
plt.ylim(-250, 250)

plt.show()

# this data structure represents the map
m = loadU08520Map()


def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):

	# main loop
	while not finished.is_set():
		print("Original Position: ", CozmoSimWorld._touch_only_for_experiments_get_position(simWorld))

		# - 4 -
		speeds = np.load("tests_to_adjust.npy")
		# speeds = np.load("track_speed_square10.npy")
		# speeds = np.load("track_speed_straight10.npy")
		# speeds = np.load("track_speed_spin10.npy")
		# speeds = np.load("track_speed_spin_and_move30.npy")
		# speeds = np.load("track_speed_complex40.npy")
		for i in range(0, len(speeds)):
			print()
			print("Speeds", i, ":  ", speeds[i, 0], " ", speeds[i, 1])
			print()
			simWorld.drive_wheel_motors(speeds[i, 0], speeds[i, 1])
			time.sleep(interval)

		simWorld.drive_wheel_motors(0, 0)
		print("Stop", end="\r\n")
		print("Final Position: ", CozmoSimWorld._touch_only_for_experiments_get_position(simWorld))
		finished.set()


interval = 0.1

# - 5 -
# current_pose = Frame2D.fromXYA(100, 500, 3.1416 / 2)
# current_pose = Frame2D.fromXYA(200, 100, 3.1416 / 2)
# current_pose = Frame2D.fromXYA(100, 200, 0)
current_pose = Frame2D.fromXYA(300, 200, 0)
# current_pose = Frame2D.fromXYA(200, 400, 3.1416 / 2)


def cozmo_update_position(simWorld: CozmoSimWorld, finished):
	global current_pose
	while not finished.is_set():
		# lspeed = robot.left_wheel_speed.speed_mmps
		# rspeed = robot.right_wheel_speed.speed_mmps
		lspeed = simWorld.left_wheel_speed()
		rspeed = simWorld.right_wheel_speed()
		delta = track_speed_to_pose_change(lspeed, rspeed, interval)
		current_pose = Frame2D.mult(current_pose, delta)
		time.sleep(interval)


def cozmo_program(simWorld: CozmoSimWorld):
	finished = threading.Event()
	print("Starting simulation. Press Q to exit", end="\r\n")
	threading.Thread(target=runWorld, args=(simWorld,finished)).start()
	threading.Thread(target=WaitForChar, args=(finished,'[Qq]')).start()
	threading.Thread(target=runCozmoMainLoop, args=(simWorld,finished)).start()
	threading.Thread(target=cozmo_update_position, args=(simWorld,finished)).start()
	# running the plot loop in a thread is not thread-safe because matplotlib
	# uses tkinter, which in turn has a threading quirk that makes it
	# non-thread-safe outside the python main program.
	# See https://stackoverflow.com/questions/14694408/runtimeerror-main-thread-is-not-in-main-loop

	runPlotLoop(m, simWorld, finished)


# NOTE: this code allows to specify the initial position of Cozmo on the map
# simWorld = CozmoSimWorld(m,Frame2D.fromXYA(200,200,0))
simWorld = CozmoSimWorld(m, current_pose)
cozmo_program(simWorld)
