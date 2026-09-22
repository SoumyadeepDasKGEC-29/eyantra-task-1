'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 1A of Niti Vahan (NV) Theme.
*
*  Filename:         ackermann_steering.py
*  Created:          2026
*  Last Modified:
*  Author:           e-Yantra Team
*
*  You are ONLY allowed to write your code inside the block marked
*  "ADD YOUR IMPLEMENTATION HERE". Do not change anything outside it - the
*  evaluation script relies on the rest of this file staying as it is.
*
*****************************************************************************************
'''

# Team ID:          < 4417 >
# Author List:      < SOUMYADEEP DAS >
# Filename:         ackermann_steering.py
# Functions:        ackermann_wheel_angles
# Global variables: < NONE >


####################### IMPORT MODULES #######################
import math
import numpy as np
##############################################################


#################### VEHICLE CONSTANTS #######################
WHEELBASE = 0.120           # L: distance between front and rear axle centrelines
TRACK_WIDTH = 0.110         # W: distance between left and right wheel centre
WHEEL_OFFSET = 0.0275       # O: distance between kingpin axis and wheel centre.
##############################################################


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################


def ackermann_wheel_angles(delta):
    '''
    Purpose:
    ---
    Convert a single virtual steering angle into the two real front-wheel
    angles, per the Ackermann geometry.

    Input Arguments:
    ---
    `delta` :   [ float ]
        Steering angle of the virtual centred front wheel, in radians.

    Returns:
    ---
    `left_angle`  : [ float ]
    `right_angle` : [ float ]
        The two real front-wheel steering angles, in radians, using the
        same sign convention as delta.

    REMEMBER:
    ---
    WHEEL_OFFSET changes the effective half-track width inside each wheel's triangle.
    '''

    # Calculate the effective half-track width due to the wheel offset

    effective_half_track = (TRACK_WIDTH / 2.0) - WHEEL_OFFSET


    # Handle the case when delta is zero to avoid division by zero

    if delta == 0.0:
        return 0.0, 0.0

    #take absolute value to avoid negative turn radius

    abs_delta = abs(delta)


    turn_radius = WHEELBASE / math.tan(abs_delta)


    outer_angle = math.atan2(WHEELBASE, turn_radius + effective_half_track)
    inner_angle = math.atan2(WHEELBASE, turn_radius - effective_half_track)

    #Use the sign of delta to determine that the car is turning left or right and assign the angles accordingly
    #Letf --> (+) , Right --> (-)
    #and assign the outer and inner angles to the left and right wheels accordingly
    
    if delta >0:    #delta positive --> Left turn --> Left wheel becomes inner wheel
        left_angle = inner_angle
        right_angle = outer_angle
    else:           #delta Negative --> right turn --> right wheel becomes inner wheel
        left_angle = -outer_angle
        right_angle = -inner_angle  

    
    return left_angle, right_angle




##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

if __name__ == "__main__":

    test_angles = np.arange(-0.35, 0.35, 0.05)

    for d in test_angles:
        left, right = ackermann_wheel_angles(d)
        print(f"delta={d}  ->  left={left}, right={right}")