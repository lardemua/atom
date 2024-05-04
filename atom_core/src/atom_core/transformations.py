#!/usr/bin/env python3
"""
A set of functions related to geometric transformations
"""

# -------------------------------------------------------------------------------
# --- IMPORTS (standard, then third party, then my own modules)
# -------------------------------------------------------------------------------

from tf.transformations import euler_from_matrix, quaternion_matrix

# Standard imports

import cv2
import atom_core.ros_numpy
import open3d as o3d
import numpy as np
from numpy.linalg import norm

# ROS imports
from image_geometry import PinholeCameraModel
from rospy_message_converter import message_converter

# Atom imports
from atom_calibration.collect.label_messages import convertDepthImage16UC1to32FC1
from atom_core.dataset_io import getPointCloudMessageFromDictionary, read_pcd

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def compareTransforms(t1, t2):
    """Compares two different transformations and returns the following metrics:


    Args:
        t1: transformation 1
        t2: transformation 2
    """

    if type(t1) == 'dict' and type(t2) == 'dict':
        quat1 = t1['quat']

        M1 = quaternion_matrix(quat1)
        print(M1)
        exit(0)

    # Method: We will use the following method. If T1 and T2 are the same, then multiplying one by the inverse of the other will produce and identity matrix, with zero translation and rotation. So we will do the multiplication and then evaluation the amount of rotation and translation in the resulting matrix.
    # print('Comparing \nt1=\n' + str(t1) + '\n\nt2=\n' + str(t2))

    t_delta = np.dot(np.linalg.inv(t1), t2)
    # print('t_delta = ' + str(t_delta))

    rotation_delta = t_delta[0:3, 0:3]
    roll, pitch, yaw = euler_from_matrix(rotation_delta)

    translation_delta = t_delta[0:3, 3]
    # print('translation_delta = ' + str(translation_delta))
    x, y, z = translation_delta

    # global metrics
    translation_error = float(abs(np.average(translation_delta)))

    rotation_error = float(np.average([abs(roll), abs(pitch), abs(yaw)]))
    # TODO How to compute aggregate rotation error? For now we use the average

    # print('Error is translation=' + str(translation_error) + ' rotation=' + str(rotation_error) + ' x=' + str(x) +
    #   ', y=' + str(y) + ', z=' + str(z) + ', roll=' + str(roll) + ', pitch=' + str(pitch) + ', yaw=' + str(yaw))

    return translation_error, rotation_error, x, y, z, roll, pitch, yaw
