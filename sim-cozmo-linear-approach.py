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

# current_pose = Frame2D.fromXYA(500, 300, -3.1416 / 2)
# current_pose = Frame2D.fromXYA(200, 500, 0)
current_pose = Frame2D.fromXYA(500, 100, 0)
x0 = current_pose.toXYA()[0]
y0 = current_pose.toXYA()[1]
a0 = current_pose.toXYA()[2]

# TODO allow the target to be chosen as console parameter
# target_pose = Frame2D.fromXYA(100, 100, -3.1416/2)  # 3.1416
# target_pose = Frame2D.fromXYA(500, 200, 0)
target_pose = Frame2D.fromXYA(100, 300, 0)
x1 = target_pose.toXYA()[0]
y1 = target_pose.toXYA()[1]
a1 = target_pose.toXYA()[2]


def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):
	global current_pose
	global target_pose

	while not finished.is_set():
		# TODO --- Rotation and move forward seems to be OK but precision is not great
		# TODO --- Final rotation needs to be done now to complete linear approach
		inv_current_pose = current_pose.inverse()
		relative_target = inv_current_pose.mult(target_pose)

		rel_tag = relative_target.toXYA()
		x = rel_tag[0]
		y = rel_tag[1]
		a = rel_tag[2]
		d = math.sqrt(x * x + y * y)  # distance between current position and target position

		print("relative_target"+str(relative_target), end="\r\n")
		velocity = target_pose_to_velocity_linear(current_pose, relative_target)
		print("velocity"+str(velocity), end="\r\n")
		track_speed = velocity_to_track_speed(velocity[0],velocity[1])
		print("trackSpeedCommand"+str(track_speed), end="\r\n")
		lspeed = simWorld.left_wheel_speed()
		rspeed = simWorld.right_wheel_speed()
		print("track_speed"+str([lspeed, rspeed]), end="\r\n")
		delta = track_speed_to_pose_change(lspeed, rspeed, interval)
		current_pose = current_pose.mult(delta)
		print("current_pose"+str(current_pose), end="\r\n")
		print("target_pose" + str(target_pose), end="\r\n")
		print()
		simWorld.drive_wheel_motors(track_speed[0], track_speed[1])
		time.sleep(interval)

		#if

		if d < 70:  # and math.fabs(a) <= 0.15:
			finished.set()



def cozmo_program(simWorld: CozmoSimWorld):
	finished = threading.Event()
	print("Starting simulation. Press Q to exit", end="\r\n")
	threading.Thread(target=runWorld, args=(simWorld,finished)).start()
	threading.Thread(target=WaitForChar, args=(finished,'[Qq]')).start()
	threading.Thread(target=runCozmoMainLoop, args=(simWorld,finished)).start()
	# running the plot loop in a thread is not thread-safe because matplotlib
	# uses tkinter, which in turn has a threading quirk that makes it
	# non-thread-safe outside the python main program.
	# See https://stackoverflow.com/questions/14694408/runtimeerror-main-thread-is-not-in-main-loop
    
	runPlotLoop(m, simWorld, finished)


# NOTE: this code allows to specify the initial position of Cozmo on the map
simWorld = CozmoSimWorld(m, current_pose)

cozmo_program(simWorld)

		

