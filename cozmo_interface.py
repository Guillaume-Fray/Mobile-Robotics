#!/usr/bin/env python3

from frame2d import Frame2D
import math
import numpy as np


wheelDistance = 81  # or 82

# TODO find sensible noise amplitudes for motor model
cozmoOdomNoiseX = 0.01
cozmoOdomNoiseY = 0.01
cozmoOdomNoiseTheta = 0.01


# Forward kinematics: compute coordinate frame update as Frame2D from left/right track speed and time of movement
def track_speed_to_pose_change(left, right, time):
    # when cozmo goes straight
    if left == right:
        frame = Frame2D.fromXYA(left*time, 0., 0.)

    # when cozmo turns (+ when turns left, - when turns right --> trigonometric direction = anticlockwise)
    else:
        theta = ((right - left)*time) / wheelDistance
        r = ((left + right)*time) / (2 * theta)
        frame = Frame2D.fromXYA(r*math.sin(theta), -r*(math.cos(theta)-1), theta)

    # createStraightTrackSpeeds()
    return frame


# Differential inverse kinematics: compute left/right track speed from desired angular and forward velocity
def velocity_to_track_speed(forward, angular):

    # when cozmo goes straight
    if angular == 0:
        left = forward
        right = forward

    # when cozmo turns
    else:
        left = forward - ((angular*wheelDistance)/2)
        right = forward + ((angular*wheelDistance)/2)

    return [left, right]


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(relative_target: Frame2D):

    s = 30  # appears to be a reasonable experimental speed

    target_position = Frame2D.toXYA(relative_target)
    x = target_position[0]
    y = target_position[1]
    a = target_position[2]

    if x == 0 and y == 0 and a != 0:
        angular = a
        velocity = 0

    elif a == 0:
        angular = 0
        velocity = s

    else:
        angular = a
        velocity = s

    return [velocity, angular]


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement by means of cubic spline interpolation 
def target_pose_to_velocity_spline(relative_target: Frame2D):

    arc_length = 1  # TODO --- Choose arc_length --- to be experimented further once function is fixed
    s = 30

    target_position = Frame2D.toXYA(relative_target)
    x = target_position[0]
    y = target_position[1]
    a = target_position[2]

    vy = 0  # TODO --- vy = final velocity's y component. Is it not = to 0 for small angular velocity values???
    k = (2*((3*y)-(s*vy)))/(s*s)  # k = curvature = radius of osculating circle
    velocity = arc_length  # forward velocity
    angular = arc_length*k  # angular velocity

    return [velocity, angular]


# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific
# measure position (relative to robot frame)
def cube_sensor_model(true_cube_position, visible, measured_position):
    return 1.0

