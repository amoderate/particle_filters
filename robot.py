
from math import *
import random

# ------------------------------------------------
#
# this is the robot class
#
max_steering_angle = pi / 4.0 # This is the to simulate limitations of a real car
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"

class Robot():

# --------
#
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

    # --------
    # init:
    #    creates robot and initializes location/orientation
    #

    def __init__(self, length = 20.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.beering_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero

    # --------
    # set:
    #    sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    # --------
    # set_noise:
    #    sets the noise parameters
    #
    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.beering_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #

    def measurement_prob(self, measurements):

        # calculate the correct measurement
        predicted_measurements = self.sense(0) # Our sense function took 0 as an argument to switch off noise.


        # compute errors
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate


            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / (self.beering_noise ** 2) / 2.0) /
                      sqrt(2.0 * pi * (self.beering_noise ** 2)))

        return error

    def __repr__(self): #allows us to print robot attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y),
                                                str(self.orientation))

    def move(self, motion, tolerance = 0.001): # Do not change the name of this function

        steering = motion[0]
        distance = motion[1]


        if abs(steering) > max_steering_angle:
            raise ValueError, 'Exceeded max steering angle'

        if distance < 0.0:
            raise ValueError, 'This Robot cannot move backwards'

        res = Robot()
        res.length = self.length
        res.beering_noise = self.beering_noise
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise

        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        turn = tan(steering2) * distance / res.length

        if abs(turn) < tolerance:
            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:

            radius = distance / turn
            cx = self.x - (sin(self.orientation)* radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)


        return res



    # --------
    # sense:
    #

    def sense(self, add_noise = 1): 
        Z = []
        for i in range(len(landmarks)):
            bearing = atan2(landmarks[i][0] - self.y,
                            landmarks[i][1] - self.x) - self.orientation
            if add_noise:
                bearing += random.gauss(0.0, self.beering_noise)
            bearing %= 2.0 * pi
            Z.append(bearing)
        return Z

