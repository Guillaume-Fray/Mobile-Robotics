#!/usr/bin/env python3

# Copyright (c) 2019 Matthias Rolf, Oxford Brookes University

'''

'''

import cozmo
from cozmo.util import degrees, Pose, distance_mm, speed_mmps
import numpy as np
from frame2d import Frame2D
from cozmo_interface import wheelDistance, target_pose_to_velocity_linear,velocity_to_track_speed,\
    track_speed_to_pose_change
import time
import math


current_pose = Frame2D()
interval = 0.1

print('Choose target position. (x, y, angle) \n')
print(' x --> ')
x1 = float(input())
print('\n y --> ')
y1 = float(input())
print('\n angle --> ')
a1 = float(input())
target_pose = Frame2D.fromXYA(x1, y1, a1)


def cozmo_drive_to_target(robot: cozmo.robot.Robot):
    global current_pose
    global target_pose
    while True:

        inv_current_pose = current_pose.inverse()
        relative_target = inv_current_pose.mult(target_pose)

        rel_tag = relative_target.toXYA()
        x = rel_tag[0]
        y = rel_tag[1]
        d = math.sqrt(x * x + y * y)  # distance between current position and target position

        print('\n distance to target: ', d, '\n')
        print("relative_target"+str(relative_target))
        velocity = target_pose_to_velocity_linear(relative_target)
        print("velocity"+str(velocity))
        track_speed = velocity_to_track_speed(velocity[0], velocity[1])
        print("trackSpeedCommand"+str(track_speed))
        lspeed = robot.left_wheel_speed.speed_mmps
        rspeed = robot.right_wheel_speed.speed_mmps
        print("track_speed"+str([lspeed, rspeed]))
        delta = track_speed_to_pose_change(lspeed, rspeed, interval)
        current_pose = delta.mult(current_pose)
        print("current_pose"+str(current_pose))
        print()
        robot.drive_wheel_motors(l_wheel_speed=track_speed[0], r_wheel_speed=track_speed[1])
        time.sleep(interval)


cozmo.run_program(cozmo_drive_to_target)
