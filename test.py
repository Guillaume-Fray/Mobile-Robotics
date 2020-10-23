from frame2d import Frame2D
import numpy as np
import cozmo_interface as ci
left11 = 3.5
left21 = 6.
right12 = 0

frame = Frame2D



#print(ci.target_pose_to_velocity_linear(frame))
#print(ci.track_speed_to_pose_change(frame))
print(frame())

