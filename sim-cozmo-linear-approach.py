#!/usr/bin/env python3


import asyncio
import cozmo
from frame2d import Frame2D 
from map import CozmoMap, plotMap, loadU08520Map, Coord2D
from matplotlib import pyplot as plt
from cozmo_interface import track_speed_to_pose_change
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear, velocity_to_track_speed,\
	track_speed_to_pose_change
from mcl_tools import *
from cozmo_sim_world import *
from cozmo_sim_world_plot import *
from terminal_reader import WaitForChar
from gaussian import Gaussian, GaussianTable, plotGaussian
import math
import numpy as np
import threading
import time


# this data structure represents the map
m = loadU08520Map()


interval = 0.1

print('Choose initial position. (x, y, angle) \n')
print(' x --> ')
x0 = float(input())
print('\n y --> ')
y0 = float(input())
print('\n angle --> ')
a0 = float(input())

current_pose = Frame2D.fromXYA(x0, y0, a0)

# current_pose = Frame2D.fromXYA(100, 600, 0)
# current_pose = Frame2D.fromXYA(200, 400, 0)
# current_pose = Frame2D.fromXYA(500, 100, 0)

print('Choose target position. (x, y, angle) \n')
print(' x --> ')
x1 = float(input())
print('\n y --> ')
y1 = float(input())
print('\n angle --> ')
a1 = float(input())

target_pose = Frame2D.fromXYA(x1, y1, a1)
# target_pose = Frame2D.fromXYA(400, 100, -3.1416/2)
# target_pose = Frame2D.fromXYA(400, 100, 0)
# target_pose = Frame2D.fromXYA(100, 300, 0)


def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):
	global current_pose
	global target_pose
	on_target = False
	d = 0

	while not finished.is_set():
		inv_current_pose = current_pose.inverse()
		relative_target = inv_current_pose.mult(target_pose)

		rel_tag = relative_target.toXYA()
		x = rel_tag[0]
		y = rel_tag[1]
		a = rel_tag[2]
		print()
		print('On target: ', on_target)
		print()
		if not on_target:
			d = math.sqrt(x * x + y * y)  # distance between current position and target position
		else:
			d = d

		velocity = target_pose_to_velocity_linear(relative_target)
		track_speed = velocity_to_track_speed(velocity[0], velocity[1])
		lspeed = simWorld.left_wheel_speed()
		rspeed = simWorld.right_wheel_speed()
		delta = track_speed_to_pose_change(lspeed, rspeed, interval)
		current_pose = current_pose.mult(delta)

		simWorld.drive_wheel_motors(track_speed[0], track_speed[1])
		time.sleep(interval)

		print("velocity"+str(velocity), end="\r\n")
		print("trackSpeedCommand" + str(track_speed), end="\r\n")
		print("track_speed" + str([lspeed, rspeed]), end="\r\n")
		print("current_pose"+str(current_pose), end="\r\n")
		print("target_pose" + str(target_pose), end="\r\n")
		print("relative_target"+str(relative_target), end="\r\n")
		print()

		if d < 60:
			on_target = True
			# 0.035 = 2 degrees (+/-)
			print('relative angle a = ', a)
		if on_target and math.fabs(a) <= 0.007:
			finished.set()


def cozmo_program(simWorld: CozmoSimWorld):
	finished = threading.Event()
	print("Starting simulation. Press Q to exit", end="\r\n")
	threading.Thread(target=runWorld, args=(simWorld, finished)).start()
	threading.Thread(target=WaitForChar, args=(finished, '[Qq]')).start()
	threading.Thread(target=runCozmoMainLoop, args=(simWorld, finished)).start()
	# running the plot loop in a thread is not thread-safe because matplotlib
	# uses tkinter, which in turn has a threading quirk that makes it
	# non-thread-safe outside the python main program.
	# See https://stackoverflow.com/questions/14694408/runtimeerror-main-thread-is-not-in-main-loop
	runPlotLoop(m, simWorld, finished)


# NOTE: this code allows to specify the initial position of Cozmo on the map
simWorld = CozmoSimWorld(m, current_pose)

cozmo_program(simWorld)
