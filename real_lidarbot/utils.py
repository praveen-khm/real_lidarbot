#!/usr/bin/python3

'''
    Utility functions for converting euler angles to quaternion and vice versa. 
    
    Adapted from https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html.
'''

from math import cos, sin, asin, atan2
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ''' Returns a quaternion from euler angles. '''

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def euler_from_quaternion(x, y, z, w):
    ''' Returns euler angles (with rotation sequence ZYX) from a quaternion. 

        angles[0] is the x-axis rotation
        angles[1] is the y-axis rotation
        angles[2] is the z-axis rotation
    '''
    angles = np.empty((3, ))

    # x-axis rotation
    angles[0] = atan2(2*(w * x + y * z), 1 - 2*(x ** 2 + y ** 2))

    # y-axis rotation
    angles[1] = asin(2*(w * y - z * x))

    # z-axis rotation
    angles[2] = atan2(2 * (w * z + x * y), 1 - 2*(y ** 2 + z ** 2))

    return angles