# import some resources
import numpy as np
import matplotlib.pyplot as plt
import random

import robot_class

world_size         = 10.0    # size of world (square)
measurement_range  = 100.0     # range at which we can sense landmarks
motion_noise       = 0.2      # noise in robot motion
measurement_noise  = 0.2      # noise in the measurements

# instantiate a robot, r
r = robot_class.robot(world_size, measurement_range, motion_noise, measurement_noise)

# print out the location of r
print(r)

# import helper function
from helpers import display_world

# define figure size
plt.rcParams["figure.figsize"] = (5,5)

# call display_world and display the robot in it's grid world
#display_world(int(world_size), [r.x, r.y])

# choose values of dx and dy (negative works, too)
dx = 1
dy = 2
r.move(dx, dy)

# print out the exact location
print(r)

# display the world after movement, not that this is the same call as before
# the robot tracks its own movement
#display_world(int(world_size), [r.x, r.y])

# create any number of landmarks
num_landmarks = 10
r.make_landmarks(num_landmarks)

# print out our robot's exact location
print(r)

# print the locations of the landmarks
print('Landmark locations [x,y]: ', r.landmarks)

measurements = r.sense()

# this will print out an empty list if `sense` has not been implemented
print(measurements)

# display the world including these landmarks
display_world(int(world_size), [r.x, r.y], r.landmarks)
