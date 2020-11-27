#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''


import numpy as np
from math import pi, cos, sin

from matplotlib import pyplot as plt

from frame2d import Frame2D

from gaussian import Gaussian, plotGaussian

from cozmo_interface import track_speed_to_pose_change, cozmoOdomNoiseX, cozmoOdomNoiseY, cozmoOdomNoiseTheta


np.random.seed(seed=1)


def createStraightTrackSpeeds():
	trackSpeeds = []
	for x in range(0, 300):
		trackSpeeds.append([-10, -10])
	for x in range(0, 400):
		trackSpeeds.append([10, 10])
	return np.array(trackSpeeds)


def createCurvyTrackSpeeds():
	trackSpeeds = []
	for x in range(0, 200):
		trackSpeeds.append([10, 25])
	for x in range(0, 200):
		trackSpeeds.append([10, 10])
	for x in range(0, 200):
		trackSpeeds.append([25, 10])
	return np.array(trackSpeeds)


def createRectangularTrackSpeeds():
	trackSpeeds = []
	for x in range(0, 100):
		trackSpeeds.append([20, 20])
	for x in range(0, 100):
		trackSpeeds.append([-7, 7])
	for x in range(0, 100):
		trackSpeeds.append([20, 20])
	for x in range(0, 100):
		trackSpeeds.append([-7, 7])
	for x in range(0, 200):
		trackSpeeds.append([20, 20])
	return np.array(trackSpeeds)


def createTurningTrackSpeeds():
	trackSpeeds = []
	for x in range(0, 10000):
		trackSpeeds.append([-10, 10])
	for x in range(0, 200):
		trackSpeeds.append([10, 10])
	return np.array(trackSpeeds)


# create track speed data
trackSpeeds = createStraightTrackSpeeds()
# trackSpeeds = createCurvyTrackSpeeds()
# trackSpeeds = createRectangularTrackSpeeds()
# trackSpeeds = createTurningTrackSpeeds()


# Save and load file
np.save("trackSpeeds4", trackSpeeds)
# speeds = np.load("trackSpeeds555.npy")


# Setup Noise model 
zero3 = np.zeros([3])
zero33 = np.zeros([3, 3])
xyaNoiseVar = np.diag([cozmoOdomNoiseX, cozmoOdomNoiseY, cozmoOdomNoiseTheta])
xyaNoise = Gaussian(zero3, xyaNoiseVar)

numParticles = 100
# 3D array for positions of particles. Index 1: Particle index. Index 2: timestep. Index 3: x,y,a
poseVecs = np.zeros([numParticles, len(trackSpeeds)+1, 3])
# 2D array for frames of particles. Index 1: Particle index. Index 2: timestep.
poseFrames = []
for run in range(0, numParticles):
	poseFrames.append([Frame2D()])

# Run simulations
for t in range(0, len(trackSpeeds)):
	# Deterministic model
	poseChange = track_speed_to_pose_change(trackSpeeds[t, 0], trackSpeeds[t, 1], 0.1)
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
for t in range(-1, len(trackSpeeds)-1, 50):
	print(t)
	print(np.mean(poseVecs[:, t, :]))
	empiricalG = Gaussian.fromData(poseVecs[:, t, :])
	print(empiricalG)
	plotGaussian(empiricalG, color="red")

# scatter plot final step particle polulation
plt.scatter(poseVecs[:, len(trackSpeeds)-1, 0], poseVecs[:, len(trackSpeeds)-1, 1], color="red", zorder=3, s=10, alpha=0.5)

plt.ylim(-200, 400)
plt.xlim(-450, 250)

plt.show()
