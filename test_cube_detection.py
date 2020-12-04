

from cozmo_interface import track_speed_to_pose_change
from cozmo_sim_world_plot import *
import threading
import time


# this data structure represents the map
m = loadU08520Map()


def runCozmoMainLoop(simWorld: CozmoSimWorld, finished):


	# main loop
	while not finished.is_set():
		robot_pose = CozmoSimWorld._touch_only_for_experiments_get_position(simWorld)
		print()
		print("Cozmo's position: ", CozmoSimWorld._touch_only_for_experiments_get_position(simWorld))
		print()
		simWorld.drive_wheel_motors(0, 0)
		time.sleep(2)

		cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
		i = 1
		for cubeID in cubeIDs:
			cube_pose = simWorld.cube_pose_global(cubeID)
			is_visible = simWorld.cube_is_visible(cubeID)
			related_pose = robot_pose.inverse().mult(cube_pose)
			x = math.fabs(related_pose.mat[0, 2])
			y = math.fabs(related_pose.mat[1, 2])
			print("Cube ID", i, " Visible --> ", is_visible)
			print("   pose: " + str(cube_pose))
			print("   Cube relative pose (2D): " + str(related_pose))
			distance = math.sqrt(x*x + y*y)
			print("   Robot-Cube distance: " + str(distance))
			print()
			i = i + 1

		finished.set()


interval = 0.1
current_pose = Frame2D.fromXYA(460, 500, 25*(math.pi / 180))  # 35*(math.pi / 180), -(57+15)*(math.pi / 180)


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
