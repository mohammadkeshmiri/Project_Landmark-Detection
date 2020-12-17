# import some resources
import numpy as np
import matplotlib.pyplot as plt
import random

import robot_class

def initialize_constraints(N, num_landmarks, world_size):
    ''' This function takes in a number of time steps N, number of landmarks, and a world_size,
        and returns initialized constraint matrices, omega and xi.'''
    
    ## Recommended: Define and store the size (rows/cols) of the constraint matrix in a variable
    
    ## TODO: Define the constraint matrix, Omega, with two initial "strength" values
    ## for the initial x, y location of our robot
    omega = np.zeros(shape=(2*N + 2*num_landmarks,2*N + 2*num_landmarks))
    ## TODO: Define the constraint *vector*, xi
    ## you can assume that the robot starts out in the middle of the world with 100% confidence
    xi = np.zeros((2*N + 2*num_landmarks, 1))

    #the first robot position is in the middle of the word
    omega[0][0] = omega[1][1] = 1
    xi[0][0] = xi[1][0] = world_size / 2 
    
    return omega, xi

## TODO: Complete the code to implement SLAM

## slam takes in 6 arguments and returns mu, 
## mu is the entire path traversed by a robot (all x,y poses) *and* all landmarks locations
def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):
    
    ## TODO: Use your initilization to create constraint matrices, omega and xi
    omega, xi = initialize_constraints(N, num_landmarks, world_size)

    ## TODO: Iterate through each time step in the data
    ## get all the motion and measurement data as you iterate
    for time_step in range(len(data)):

        motion = data[time_step][1]
        measurements = data[time_step][0]

        omega[2* time_step][2 * time_step] += 1 / motion_noise#x_i-1
        omega[2* time_step][2*time_step + 2] += -1 / motion_noise# x_i
        omega[2*time_step + 2][2*time_step] += -1 / motion_noise# x_i-1
        omega[2*time_step + 2][2*time_step + 2] += 1 / motion_noise# x_i

        omega[2* time_step + 1][2 * time_step + 1] += 1 / motion_noise#y_i-1
        omega[2* time_step + 1][2*time_step + 3] += -1 / motion_noise# y_i
        omega[2*time_step + 3][2*time_step + 1] += -1 / motion_noise# y_i-1
        omega[2*time_step + 3][2*time_step + 3] += 1 / motion_noise# y_i

        # xi motion

        xi[2*time_step][0] += -motion[0] / motion_noise#x_i-1
        xi[2*time_step + 2][0] += motion[0] / motion_noise#x_i

        xi[2*time_step + 1][0] += -motion[1] / motion_noise#y_i-1
        xi[2*time_step + 3][0] += motion[1] / motion_noise#y_i

        # landmarks
        for measurement in measurements:
            omega[2*time_step][2*time_step] += 1 / measurement_noise# x_i
            omega[2*time_step][2*N + 2*measurement[0]] += -1 / measurement_noise# l_i
            omega[2*N+ 2*measurement[0]][2*time_step] += -1 / measurement_noise # x_i
            omega[2*N + 2*measurement[0]][2*N + 2*measurement[0]] += 1 / measurement_noise# l_i

            # xi landmark
            xi[2*time_step][0] += -measurement[1] / measurement_noise
            xi[2*N + 2*measurement[0]][0] += measurement[1] / measurement_noise

            omega[2*time_step + 1][2*time_step + 1] += 1 / measurement_noise# y_i
            omega[2*time_step + 1][2*N + 2*measurement[0]+1] += -1 / measurement_noise# l_i
            omega[2*N + 2*measurement[0]+1][2*time_step +1] += -1 / measurement_noise# y_i
            omega[2*N + 2*measurement[0]+1][2*N + 2*measurement[0]+1] += 1 / measurement_noise# l_i

            # xi landmark
            xi[2*time_step+1][0] += -measurement[2] / measurement_noise
            xi[2*N + 2*measurement[0] + 1][0] += measurement[2] / measurement_noise

    ## TODO: update the constraint matrix/vector to account for all *measurements*
    ## this should be a series of additions that take into account the measurement noise
            
    ## TODO: update the constraint matrix/vector to account for all *motion* and motion noise
    
    ## TODO: After iterating through all the data
    ## Compute the best estimate of poses and landmark positions
    ## using the formula, omega_inverse * Xi
    mu = np.dot(np.linalg.inv(omega), xi)
    
    return mu # return `mu`