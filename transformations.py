import numpy as np
import math


def rotation_y(q):
    '''Returns matrix representing a rotation about the y axis'''
    rmatrix = np.asarray([[math.cos(q), 0.0, math.sin(q)], [0.0, 1.0, 0.0], [
                         math.sin(q) * -1, 0.0, math.cos(q)]])
    return rmatrix


def rotation_z(q):
    '''Returns matrix representing a rotation about the z axis'''
    rmatrix = np.asarray([[math.cos(q), math.sin(q) * -1, 0.0],
                         [math.sin(q), math.cos(q), 0.0], [0.0, 0.0, 1.0]])
    return rmatrix


def rotation_x(q):
    '''Returns matrix representing a rotation about the z axis'''
    rmatrix = np.asarray([[1.0, 0.0, 0.0], [0.0, math.cos(
        q), math.sin(q) * -1], [0.0, math.sin(q), math.cos(q)]])
    return rmatrix


def generate_transform(rotmat, displacement):
    '''Returns matrix representing a transformation consisting of a rotation about a joint and a translation along the length of a link'''
    transformed = np.zeros((4, 4))
    transformed[:3, :3] = rotmat
    transformed[:3, 3] = displacement
    transformed[3, 3] = 1.0
    return transformed
