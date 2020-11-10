#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''

import cozmo

from cozmo.util import degrees, Pose, distance_mm, speed_mmps

import numpy as np

from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear,velocity_to_track_speed,track_speed_to_pose_change

import time



current_pose=Frame2D()


interval = 0.1


def cozmo_update_position(robot: cozmo.robot.Robot):
    global current_pose
    while True:
        lspeed = robot.left_wheel_speed.speed_mmps
        rspeed = robot.right_wheel_speed.speed_mmps
        delta = track_speed_to_pose_change(lspeed,rspeed,interval)
        print(lspeed, " ", rspeed, " ", current_pose, " ", delta)
        # TODO -- done? (just one line?)
        current_pose = Frame2D.mult(current_pose, delta)
        time.sleep(interval)


cozmo.run_program(cozmo_update_position, use_3d_viewer=True)

