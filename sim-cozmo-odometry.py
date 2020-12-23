#!/usr/bin/env python3


import asyncio

import cozmo

from frame2d import Frame2D 
from map import CozmoMap, plotMap, loadU08520Map, Coord2D
from matplotlib import pyplot as plt
from cozmo_interface import track_speed_to_pose_change
from cozmo_interface import wheelDistance
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


def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):

	#time.sleep(4)

	print("\n")
	print("\n")
	print("\n")
	print("Wheel distance: ", wheelDistance)
	print("\n")
	print("\n")

	# main loop
	while not finished.is_set():
		print("real start current position: ", CozmoSimWorld._touch_only_for_experiments_get_position(simWorld))
		print("estimated start position: ", current_pose)
		print("\n")
		print("\n")

		# print("Straight", end="\r\n")
		# time.sleep(8)
		# simWorld.drive_wheel_motors(30, 30)
		# time.sleep(30)
		# time.sleep(20)

		# print("Turn", end="\r\n")
		# simWorld.drive_wheel_motors(20, 13)
		# time.sleep(23)

		# print("Circle", end="\r\n")
		simWorld.drive_wheel_motors(20, 8)
		time.sleep(53)

		# print("Stop", end="\r\n")
		simWorld.drive_wheel_motors(0, 0)
		time.sleep(6)
		print("\n")
		print("\n")
		print("real final current position: ", CozmoSimWorld._touch_only_for_experiments_get_position(simWorld))
		print("estimated final position: ", current_pose)
		print("\n")
		print("\n")
		finished.set()

interval = 0.1
# currentPose = Frame2D.fromXYA(200,350,3.1416/2) # original given position
# current_pose = Frame2D.fromXYA(100, 500, 3.1416 / 2)  # for turn testing
current_pose = Frame2D.fromXYA(100, 200, 3.1416 / 2)  # for testing full circle


def cozmo_update_position(simWorld: CozmoSimWorld, finished):
	global current_pose
	while not finished.is_set():
		# lspeed = robot.left_wheel_speed.speed_mmps
		# rspeed = robot.right_wheel_speed.speed_mmps
		lspeed = simWorld.left_wheel_speed()
		rspeed = simWorld.right_wheel_speed()
		delta = track_speed_to_pose_change(lspeed, rspeed, interval)
		print(lspeed, " ", rspeed, " ", current_pose, " ", delta, end="\r\n")
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

		

