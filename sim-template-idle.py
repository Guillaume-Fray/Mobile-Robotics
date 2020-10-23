#!/usr/bin/env python3


import asyncio

import cozmo

from frame2d import Frame2D 
from map import CozmoMap, plotMap, loadU08520Map, Coord2D
from matplotlib import pyplot as plt
from cozmo_interface import track_speed_to_pose_change
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
m=loadU08520Map()



def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):

	time.sleep(5)

	# main loop
	# TODO insert driving and navigation behavior HERE
	while not finished.is_set():
		print("Straight", end="\r\n")
		simWorld.drive_wheel_motors(50,50)
		time.sleep(2)
		#time.sleep(4) 
		#time.sleep(8)
		print("Turn", end="\r\n")
		simWorld.drive_wheel_motors(-20,20)
		time.sleep(3)
		print("Stop", end="\r\n")
		simWorld.drive_wheel_motors(0,0)
		time.sleep(2)


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
#simWorld = CozmoSimWorld(m,Frame2D.fromXYA(200,200,0))
simWorld = CozmoSimWorld(m,Frame2D.fromXYA(200,350,-3.1416/2))

cozmo_program(simWorld)

		

