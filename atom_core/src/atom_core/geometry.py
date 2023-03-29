import math

import cv2
import numpy as np


def distance_two_3D_points(p0, p1):
    return math.sqrt(((p0[0] - p1[0]) ** 2) + ((p0[1] - p1[1]) ** 2) + ((p0[2] - p1[2]) ** 2))


def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
    """
    p0, p1: Define the line.
    p_co, p_no: define the plane:
        p_co Is a point on the plane (plane coordinate).
        p_no Is a normal vector defining the plane direction;
             (does not need to be normalized).

    Return a Vector or None (when the intersection can't be found).
    """

    u = sub_v3v3(p1, p0)
    dot = dot_v3v3(p_no, u)

    if abs(dot) > epsilon:
        # The factor of the point between p0 -> p1 (0 - 1)
        # if 'fac' is between (0 - 1) the point intersects with the segment.
        # Otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = sub_v3v3(p0, p_co)
        fac = -dot_v3v3(p_no, w) / dot
        u = mul_v3_fl(u, fac)
        return add_v3v3(p0, u)
    else:
        # The segment is parallel to plane.
        return None


def add_v3v3(v0, v1):
    return (
        v0[0] + v1[0],
        v0[1] + v1[1],
        v0[2] + v1[2],
    )


def sub_v3v3(v0, v1):
    return (
        v0[0] - v1[0],
        v0[1] - v1[1],
        v0[2] - v1[2],
    )


def dot_v3v3(v0, v1):
    return (
        (v0[0] * v1[0]) +
        (v0[1] * v1[1]) +
        (v0[2] * v1[2])
    )


def len_squared_v3(v0):
    return dot_v3v3(v0, v0)


def mul_v3_fl(v0, f):
    return (
        v0[0] * f,
        v0[1] * f,
        v0[2] * f,
    )


def fitPlaneLTSQ(XYZ):
    (rows, cols) = XYZ.shape
    G = np.ones((rows, 3))
    G[:, 0] = XYZ[:, 0]  # X
    G[:, 1] = XYZ[:, 1]  # Y
    Z = XYZ[:, 2]
    (a, b, c), resid, rank, s = np.linalg.lstsq(G, Z)
    normal = (a, b, -1)
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return (c, normal)


def matrixToRodrigues(T):
    rods, _ = cv2.Rodrigues(T[0:3, 0:3])
    rods = rods.transpose()
    return rods[0]


def rodriguesToMatrix(r):
    rod = np.array(r, dtype=float)
    matrix = cv2.Rodrigues(rod)
    return matrix[0]


def traslationRodriguesToTransform(translation, rodrigues):
    R = rodriguesToMatrix(rodrigues)
    T = np.zeros((4, 4), dtype=float)
    T[0:3, 0:3] = R
    T[0, 3] = translation[0]
    T[1, 3] = translation[1]
    T[2, 3] = translation[2]
    T[3, 3] = 1
    return T


def translationQuaternionToTransform(trans, quat):
    matrix = quaternionMatrix(quat)
    matrix[0, 3] = trans[0]
    matrix[1, 3] = trans[1]
    matrix[2, 3] = trans[2]
    matrix[3, 3] = 1
    # print(str(matrix))
    return matrix


def quaternionMatrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    Copied from 2006, Christoph Gohlke
    """
    _EPS = np.finfo(float).eps * 4.0

    q_ = np.array(quaternion[:4], dtype=np.float64).copy()
    nq = np.dot(q_, q_)
    if nq < _EPS:
        return np.identity(4)
    q_ *= math.sqrt(2.0 / nq)
    q = np.outer(q_, q_)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (0.0,                 0.0,                 0.0, 1.0)
    ), dtype=np.float64)
