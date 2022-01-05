# stdlib
import atom_core.atom
import math
from copy import deepcopy

import chardet
import numpy as np
import ros_numpy
from scipy.spatial import distance

# 3rd-party
import OptimizationUtils.utilities as opt_utilities
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from rospy_message_converter import message_converter

# Own modules
from atom_core.dataset_io import getPointCloudMessageFromDictionary, getCvImageFromDictionaryDepth
from atom_core.geometry import distance_two_3D_points, isect_line_plane_v3
from atom_core.cache import Cache
from atom_calibration.collect.label_messages import pixToWorld, worldToPix


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

@Cache(args_to_ignore=['_dataset'])
def getPointsInSensorAsNPArray(_collection_key, _sensor_key, _label_key, _dataset):
    cloud_msg = getPointCloudMessageFromDictionary(_dataset['collections'][_collection_key]['data'][_sensor_key])
    idxs = _dataset['collections'][_collection_key]['labels'][_sensor_key][_label_key]
    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points = np.zeros((4, pc.shape[0]))
    points[0, :] = pc['x']
    points[1, :] = pc['y']
    points[2, :] = pc['z']
    points[3, :] = 1
    return points

@Cache(args_to_ignore=['_dataset'])
def getUnlabeledPointsInSensorAsNPArray(_collection_key, _sensor_key, _label_key, _dataset):
    cloud_msg = getPointCloudMessageFromDictionary(_dataset['collections'][_collection_key]['data'][_sensor_key])
    size = cloud_msg.height * cloud_msg.width
    not_idxs = _dataset['collections'][_collection_key]['labels'][_sensor_key][_label_key]
    all_idxs = np.linspace(start=0, stop=(size - 1), num=size, dtype=int).tolist()
    idxs = [idx for idx in all_idxs if idx not in not_idxs]
    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points = np.zeros((4, pc.shape[0]))
    points[0, :] = pc['x']
    points[1, :] = pc['y']
    points[2, :] = pc['z']
    points[3, :] = 1
    return points
