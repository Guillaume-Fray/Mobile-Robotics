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
    if math.fabs(left - right) < 0.5:
        frame = Frame2D.fromXYA(left * time, 0., 0.)

    # when cozmo turns (+ when turns left, - when turns right --> trigonometric direction = anticlockwise)
    else:
        theta = ((right - left) * time) / wheelDistance
        r = ((left + right) * time) / (2 * theta)
        frame = Frame2D.fromXYA(r * math.sin(theta), -r * (math.cos(theta) - 1), theta)

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
        left = forward - ((angular * wheelDistance) / 2)
        right = forward + ((angular * wheelDistance) / 2)

    return [left, right]


well_oriented = False  # global variable to stop cozmo from spinning on the spot once it is well orientated
final_orientation = False  # global variable for cozmo to adjust its orientation once it's reached final position


# Trajectory planning: given target (relative to robot frame), determine next forward/angular motion
# Implement in a linear way
# If far away and facing wrong direction: rotate to face target
# If far away and facing target: move forward
# If on target: turn to desired orientation
def target_pose_to_velocity_linear(current_pose: Frame2D, relative_target: Frame2D):
    global well_oriented
    s = 60

    rel_target_position = Frame2D.toXYA(relative_target)
    x2 = rel_target_position[0]
    y2 = rel_target_position[1]
    a2 = rel_target_position[2]
    d = math.sqrt(x2*x2 + y2*y2)  # distance between current position and target position
    print('distance = ', d)

    # cos_a = relative_target.mat[0, 0]

    cur_position = Frame2D.toXYA(current_pose)
    a1 = cur_position[2]

    alpha = math.atan2(y2, x2) - math.pi/2
    print('a1 = ', a1)
    print('a2 = ', a2)
    print('alpha = ', alpha)
    # TODO    alpha changes after every time interval when it shouldn't whether the orientation is right or not.
    # TODO    It should always be close to 0 (or to 2*pi) when target and current pose have same x or same y
    # TODO    This is why it keeps spinning!!! Once alpha is fixed, all I need to do is ensure a1 = alpha and stops
    # TODO    angular motion until cozmo reaches target
    print('\n')

    # target far away
    if d > 70:

        # facing target
        if well_oriented:
            velocity = s
            angular = 0

        # wrong orientation
        # check that the absolute difference of the cos values of alpha and a1 (current cozmo angle) < 1 degree
        # 1 degree = pi/180 = 0.01745329252 and cos(pi/180) = 0.9998476951
        elif not well_oriented and math.fabs((math.fabs(math.cos(alpha)))-(math.fabs(math.cos(a1)))) < 0.087265:
                angular = 1
                velocity = 0

        else:
            well_oriented = True
            angular = 0
            velocity = s

    # on target
    else:
        velocity = 0
        angular = 0

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
    k = (2 * ((3 * y) - (s * vy))) / (s * s)  # k = curvature = radius of osculating circle
    velocity = arc_length  # forward velocity
    angular = arc_length * k  # angular velocity

    return [velocity, angular]


# Take a true cube position (relative to robot frame). 
# Compute /probability/ of cube being (i) visible AND being detected at a specific
# measure position (relative to robot frame)
def cube_sensor_model(true_cube_position, visible, measured_position):
    return 1.0
