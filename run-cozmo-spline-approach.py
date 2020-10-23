#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''

import cozmo

from cozmo.util import degrees, Pose, distance_mm, speed_mmps

import numpy as np
from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear, velocity_to_track_speed,\
    track_speed_to_pose_change, target_pose_to_velocity_spline

import time


current_pose = Frame2D()

# TODO allow the target to be chosen as console parameter
targetPose = Frame2D.fromXYA(200, 200, 0.0*3.1416)

interval = 0.1


def cozmo_drive_to_target(robot: cozmo.robot.Robot):
    global current_pose
    global target_pose
    while True:

        # TODO determine current position of target relative to robot - the math is wrong
        target_pose = Frame2D.inverse(target_pose)
        relative_target = Frame2D.mult(current_pose, target_pose)

        print("relative_target"+str(relative_target))
        velocity = target_pose_to_velocity_spline(relative_target)
        print("velocity"+str(velocity))
        track_speed = velocity_to_track_speed(velocity[0],velocity[1])
        print("trackSpeedCommand"+str(track_speed))
        lspeed = robot.left_wheel_speed.speed_mmps
        rspeed = robot.right_wheel_speed.speed_mmps
        print("track_speed"+str([lspeed, rspeed]))
        delta = track_speed_to_pose_change(lspeed, rspeed,interval)
        current_pose = Frame2D.mult(current_pose, delta)
        print("currentPose" + str(current_pose))
        print()
        robot.drive_wheel_motors(l_wheel_speed=track_speed[0],r_wheel_speed=track_speed[1])
        time.sleep(interval)


cozmo.run_program(cozmo_drive_to_target)
