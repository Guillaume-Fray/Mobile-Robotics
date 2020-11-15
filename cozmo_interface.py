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
alpha_set = False  # global variable to prevent alpha from changing due to noise
alpha = 0  # angle between initial position and final position


def get_on_target():
    return on_target


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(current_pose: Frame2D, relative_target: Frame2D):
    global well_oriented
    global final_orientation
    global on_target
    global alpha_set
    global alpha
    difference = 0
    s = 40

    rel_target_position = Frame2D.toXYA(relative_target)
    x2 = rel_target_position[0]
    y2 = rel_target_position[1]
    a2 = rel_target_position[2]
    d = math.sqrt(x2*x2 + y2*y2)  # distance between current position and target position
    print('distance = ', d)

    cur_position = Frame2D.toXYA(current_pose)
    a1 = cur_position[2]

    # alpha is the angle between initial position and target position
    if not alpha_set:
        alpha = math.atan2(y2, x2)
        alpha_set = True
    else:
        alpha = alpha

    print('a1 = ', a1)
    print('a2 = ', a2)
    print('alpha = ', alpha)
    print('\n')

    # TODO if cozmo's initial position is turning its back to the target, x and y get inverted,
    # TODO which means that alpha becomes wrong and cozmo goes in the wrong direction!!

    # TODO anyway difference strategy not working. Needs a guard to avoid adding angles up in some cases
    # difference used to get cozmo to rotate in the adequate direction (left or right) to face target
    if not well_oriented:
        difference = alpha - a1
    else:
        difference = difference

    print('difference is: ', difference)
    print('\n')

    # target far away
    if d > 70 and not on_target:

        # facing target TODO you can remove this "if" later, the else is enough
        if well_oriented:
            velocity = s
            angular = 0

        # wrong orientation
        # ensures that cozmo rotates to face target
        # 5 degrees = pi/180 * 5 = 0.087265 rad  ||   2 degrees ~ 0.035 rad || 1 deg = 0.01745329252 rad
        elif not well_oriented and alpha > 0.035:
                angular = 0.5
                velocity = 0

        elif not well_oriented and alpha < -0.035:
                angular = -0.5
                velocity = 0

        else:
            well_oriented = True
            alpha_set = True
            angular = 0
            velocity = s

    # on target
    else:
        on_target = True
        # wrong orientation
        if not final_orientation and a2 > 0.035:
            angular = 0.5
            velocity = 0

        elif not final_orientation and a2 < -0.035:
                angular = -0.5
                velocity = 0

        # right orientation
        else:
            final_orientation = True
            angular = 0
            velocity = 0

    return [velocity, angular]


def linear(relative_target: Frame2D):
    global well_oriented
    global on_target
    rel_tag = relative_target.toXYA()
    x = rel_tag[0]
    y = rel_tag[1]
    a = rel_tag[2]

    s = 40
    d = math.sqrt(x*x + y*y)
    beta = math.atan2(y, x)
    print('distance = ', d)

    if d > 70 and not on_target and not well_oriented:
        if math.cos(beta) <= 0.035:
            angular = 0.5
            forward = 0
        else:
            well_oriented = True
            angular = 0
            forward = s

    elif well_oriented:
        angular = 0
        forward = s

    else:
        on_target = True
        angular = 0
        forward = 0

    return [forward, angular]


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement by means of cubic spline interpolation 
def target_pose_to_velocity_spline(relative_target: Frame2D):
    arc_length = 0.5  # TODO --- arc length seems ok for now ---
    s = 30

    target_position = Frame2D.toXYA(relative_target)
    x = target_position[0]
    y = target_position[1]
    a = target_position[2]

    vy = relative_target.mat[1, 0]  # vy = final velocity's y component
    k = (2 * ((3 * y) - (s * vy))) / (s * s)  # k = curvature = radius of osculating circle
    velocity = s  # forward velocity
    angular = arc_length * k  # angular velocity

    return [velocity, angular]


# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific
# measure position (relative to robot frame)
def cube_sensor_model(true_cube_position, visible, measured_position):
    return 1.0
