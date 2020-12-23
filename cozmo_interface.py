#!/usr/bin/env python3

from frame2d import Frame2D
import math
import numpy as np

wheelDistance = 80

#
cozmoOdomNoiseX = 0.01
cozmoOdomNoiseY = 0.01
cozmoOdomNoiseTheta = 0.000006


# Forward kinematics: compute coordinate frame update as Frame2D from left/right track speed and time of movement
def track_speed_to_pose_change(left, right, time):
    # when cozmo goes straight-ish
    if math.fabs(left - right) < 0.1:
        frame = Frame2D.fromXYA(left * time, 0., 0.)

    # when cozmo turns (+ when turns left, - when turns right --> trigonometric direction = anticlockwise)
    else:
        theta = ((right - left) * time) / wheelDistance
        r = ((left + right) * time) / (2 * theta)
        frame = Frame2D.fromXYA(r * math.sin(theta), -r * (math.cos(theta) - 1), theta)

    return frame


# Differential inverse kinematics: compute left/right track speed from desired angular and forward velocity
def velocity_to_track_speed(forward, angular):
    left = forward - ((angular * wheelDistance) / 2)
    right = forward + ((angular * wheelDistance) / 2)

    return [left, right]


well_oriented = False  # global variable to stop cozmo from spinning on the spot once it is well orientated
final_orientation = False  # global variable for cozmo to adjust its orientation once it's reached final position
on_target = False  # global variable for cozmo to stop when on target before rotating to final orientation


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(relative_target: Frame2D):
    global on_target
    global well_oriented
    global final_orientation
    s = 40

    rel_target_position = Frame2D.toXYA(relative_target)
    x = rel_target_position[0]
    y = rel_target_position[1]
    a = rel_target_position[2]

    d = math.sqrt(x*x + y*y)  # distance between current position and target position
    alpha = math.atan2(y, x)
    print('distance = ', d)
    print('a = ', a)
    print('alpha = ', alpha)
    print('\n')

    # target far away
    if d > 70 and not on_target:
        # wrong orientation
        # ensures that cozmo rotates to face target. +/- 2 degrees ~ 0.035 rad
        if not well_oriented and alpha > 0.007:
                angular = 0.5
                velocity = 0

        elif not well_oriented and alpha < -0.007:
                angular = -0.5
                velocity = 0

        else:
            well_oriented = True
            angular = 0
            velocity = s

    # on target
    else:
        on_target = True
        # wrong orientation
        if not final_orientation and a > 0.007:
            angular = 0.5
            velocity = 0

        elif not final_orientation and a < -0.007:
                angular = -0.5
                velocity = 0

        # right orientation
        else:
            final_orientation = True
            angular = 0
            velocity = 0

    return [velocity, angular]


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement by means of cubic spline interpolation 
def target_pose_to_velocity_spline(relative_target: Frame2D):
    arc_length = 20  # TODO --- find right arc_length and s ---
    s = 250

    target_position = Frame2D.toXYA(relative_target)
    x = target_position[0]
    y = target_position[1]

    vy = relative_target.mat[1, 0]  # vy = final velocity's y component
    k = (2 * ((3 * y) - (s * vy))) / (s * s)  # k = curvature = radius of osculating circle
    velocity = arc_length  # forward velocity
    angular = arc_length * k  # angular velocity

    return [velocity, angular]


# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific
# measure position (relative to robot frame)
def cube_sensor_model(true_cube_position, visible, measured_position):
    x = math.fabs(true_cube_position.mat[0, 2])
    y = math.fabs(true_cube_position.mat[1, 2])
    a = true_cube_position.mat[2, 2]
    distance = math.fabs(x*x + y*y)

    # should I use the measured_position for the distance? or use the mean of both?

    # Cube detection probability based on tests results
    if math.fabs(a) <= 0.4363 and 150 <= distance <= 500:
        proba = 0.95

    elif 0.4363 < math.fabs(a) <= 0.5236 and 150 <= distance <= 400:
        proba = 0.6

    elif 0.4363 < math.fabs(a) <= 0.5236 and 400 < distance <= 500:
        proba = 0.5

    elif 0.4363 < math.fabs(a) <= 0.5236 and 500 < distance <= 600:
        proba = 0.3

    elif 0.2618 < math.fabs(a) <= 0.4363 and 500 < distance <= 600:
        proba = 0.4

    elif 0.17453 < math.fabs(a) <= 0.2618 and 500 < distance <= 600:
        proba = 0.5

    elif 0 < math.fabs(a) <= 0.17453 and 500 < distance <= 600:
        proba = 0.6

    elif 0.17453 < math.fabs(a) <= 0.2618 and 600 < distance <= 670:
        proba = 0.15

    elif 0 < math.fabs(a) <= 0.17453 and 600 < distance <= 670:
        proba = 0.05

    elif math.fabs(a) <= 0.08727 and distance < 150:
        proba = 0.3

    elif 0.08727 < math.fabs(a) <= 0.17453 and distance < 150:
        proba = 0.2

    elif 0.17453 < math.fabs(a) <= 0.2618 and distance < 150:
        proba = 0.4

    elif 0.2618 < math.fabs(a) <= 0.5236 and distance < 150:
        proba = 0.4

    elif 0.2618 < math.fabs(a) <= 0.5236 and distance < 150:
        proba = 0.4

    else:
        proba = 0.01


    if visible:
        proba_visible = proba
    else:
        proba_visible = 1 - proba


    return proba_visible
