#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib
import copy
import math
import os
import pprint

# 3rd-party
import cv2
import ros_numpy
import rospy
import tf

from rospy_message_converter import message_converter
from rospy_urdf_to_rviz_converter.rospy_urdf_to_rviz_converter import urdfToMarkerArray
from std_msgs.msg import Header, ColorRGBA
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import Image, sensor_msgs, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from cv_bridge import CvBridge

from matplotlib import cm
from open3d import *
# TODO TO remove after getting all functions from there
from atom_calibration.utilities import uriReader, execute


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def genCollectionPrefix(collection_key, string):
    """ Standardized form of deriving a name with a collection related prefix. """
    return generateName(string, prefix='c' + str(collection_key), separator='_')
    # return 'c' + str(collection_key) + '_' + str(string)


def generateName(name, prefix='', suffix='', separator='_'):
    """ Standardized form of deriving a name with a prefix or a suffix with <separator> separating them. """

    if prefix:
        prefix = prefix + separator

    if suffix:
        suffix = separator + suffix
    return str(prefix) + str(name) + str(suffix)


def readXacroFile(description_file):
    # xml_robot = URDF.from_parameter_server()
    urdf_file = '/tmp/description.urdf'
    print('Parsing description file ' + description_file)
    execute('xacro ' + description_file + ' -o ' + urdf_file, verbose=True)  # create a temp urdf file
    try:
        xml_robot = URDF.from_xml_file(urdf_file)  # read teh urdf file
    except:
        raise ValueError('Could not parse description file ' + description_file)

    return xml_robot