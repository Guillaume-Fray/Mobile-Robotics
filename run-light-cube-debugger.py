#!/usr/bin/env python3


import asyncio

import cozmo

from frame2d import Frame2D 

async def cozmo_program(robot: cozmo.robot.Robot):
	while True:
		robotPose = Frame2D.fromPose(robot.pose)
		print("Robot pose: " + str(robotPose))
		cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
		for cubeID in cubeIDs:
			cube = robot.world.get_light_cube(cubeID)
			if cube is not None and cube.is_visible:
				print("Visible: " + cube.descriptive_name + " (id=" + str(cube.object_id) + ")")
				cubePose = Frame2D.fromPose(cube.pose)
				print("   pose: " + str(cubePose))
				print("   relative pose (2D): " + str(robotPose.inverse().mult(cubePose)))
		print()
		await asyncio.sleep(1)


cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(cozmo_program, use_3d_viewer=True, use_viewer=True)



