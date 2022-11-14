import numpy as np
from math import sin, cos

def get_roll_rotation_mat(angle):
    #A roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis.
    return np.array([
        [1,0,0],
        [0, cos(angle), -sin(angle)],
        [0, sin(angle),  cos(angle)]
    ])

def get_pitch_rotation_mat(angle):
    #A pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis
    return np.array([
        [ cos(angle), 0, sin(angle)],
        [0, 1, 0],
        [-sin(angle), 0, cos(angle)]
    ])

def get_yaw_rotation_mat(angle):
    #A yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis

    return np.array([
        [cos(angle), -sin(angle), 0],
        [sin(angle),  cos(angle), 0],
        [0,0,1]
    ])

def get_rotation_mat(x_angle, y_angle, z_angle):
    x_mat = get_roll_rotation_mat(x_angle)
    y_mat = get_pitch_rotation_mat(y_angle)
    z_mat = get_yaw_rotation_mat(z_angle)
    return (z_mat.dot(y_mat)).dot(x_mat)