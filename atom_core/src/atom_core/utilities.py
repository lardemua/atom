#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib
import os
import re
import math
import json
import itertools
import subprocess

# 3rd-party
import numpy as np
import rospkg
import yaml
import rospy

from colorama import Fore
from rospy_message_converter import message_converter
from sensor_msgs.msg import *
from urlparse import urlparse
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
# from open3d import * # This cannot be used. It itereferes with the Image for getMessageTypeFromTopic(topic):


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

# Check https://stackoverflow.com/questions/52431265/how-to-use-a-lambda-as-parameter-in-python-argparse
def create_lambda_with_globals(s):
    return eval(s, globals())


def filterLaunchArguments(argvs):
    # Roslaunch files send a "__name:=..." argument (and __log:=) which disrupts the argparser. The solution is to
    # filter this argv. in addition, the first argument is the node name, which should also not be given to the
    # parser.

    argvs_filtered = []
    for i, argv in enumerate(argvs):
        if (not all(x in argv for x in ['__', ':='])) and (i != 0):
            argvs_filtered.append(argv)

    return argvs_filtered


def execute(cmd, blocking=True, verbose=True):
    """ @brief Executes the command in the shell in a blocking or non-blocking manner
        @param cmd a string with teh command to execute
        @return
    """
    if verbose:
        print "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if verbose:
                print line,
            p.wait()


def resolvePath(path, verbose=False):
    """ Resolves path by replacing environment variables, common notations (e.g. ~ for home/user)"""

    path = os.path.expanduser(path)
    path = os.path.expandvars(path)
    path = os.path.abspath(path)
    path = os.path.normpath(path)
    return path


def expandToLaunchEnv(path):
    if len(path) == 0:  # if path is empty, path[0] does not exist
        return path

    if path[0] == '~':
        path = '$(env HOME)' + path[1:]

    if '$' not in path:
        return path

    evars = re.compile(r'\$(\w+|\{[^}]*\})')
    i = 0
    while True:
        m = evars.search(path, i)
        if not m:
            break

        i, j = m.span(0)
        name = m.group(1)
        if name.startswith('{') and name.endswith('}'):
            name = name[1:-1]

        tail = path[j:]
        path = path[:i] + '$(env {})'.format(name)
        i = len(path)
        path += tail

    return path


def uriReader(resource):
    uri = urlparse(str(resource))
    # print(uri)
    if uri.scheme == 'package':  # using a ros package uri
        # print('This is a ros package')
        rospack = rospkg.RosPack()
        assert (rospack.get_path(uri.netloc)), 'Package ' + uri.netloc + ' does not exist.'
        fullpath = resolvePath(rospack.get_path(uri.netloc) + uri.path)
        relpath = '$(find {}){}'.format(uri.netloc, uri.path)

    elif uri.scheme == 'file':  # local file
        # print('This is a local file')
        fullpath = resolvePath(uri.netloc + uri.path)
        relpath = fullpath
    elif uri.scheme == '':  # no scheme, assume local file
        # print('This is a local file')

        fullpath = resolvePath(uri.path)
        relpath = expandToLaunchEnv(uri.path)
    else:
        raise ValueError('Cannot parse resource "' + resource + '", unknown scheme "' + uri.scheme + '".')

    assert (os.path.exists(fullpath)), Fore.RED + fullpath + ' does not exist. Check your config.yml description file'
    return fullpath, os.path.basename(fullpath), relpath


def verifyConfig(config, template_config, upper_key=None):
    missing_keys = []
    for key in template_config:
        # print('Checking key ' + key)
        if not key in config:
            missing_keys.append(upper_key + '/' + key)
            # print(str(key) + ' is not here: ' + str(config))
        elif type(config[key]) is dict and not key == 'sensors':
            # print('key ' + key + ' is a dict')

            if upper_key is None:
                mk = verifyConfig(config[key], template_config[key], key)
            else:
                mk = verifyConfig(config[key], template_config[key], upper_key + '/' + key)
            missing_keys.extend(mk)

    return missing_keys


def loadConfig(filename, check_paths=True):
    config = loadYMLConfig(filename)

    # Check if config has all the necessary keys.
    rospack = rospkg.RosPack()
    template_file = rospack.get_path('atom_calibration') + '/templates/config.yml'
    template_config = loadYMLConfig(template_file)
    missing_parameters = verifyConfig(config, template_config)

    if missing_parameters:  # list is not empty
        raise ValueError(Fore.RED + 'Your config file ' + filename +
                         ' appears to be corrupted. These mandatory parameters are missing: ' + Fore.BLUE +
                         str(missing_parameters) + Fore.RED + '\nPerhaps you should re-run:\n' + Fore.BLUE +
                         ' rosrun <your_robot>_calibration configure' + Fore.RESET)

    # Check if description file is ok
    fullpath, name, uri = uriReader(config['description_file'])

    # Check if bag_file is ok
    fullpath, name, uri = uriReader(config['bag_file'])

    # Check if calibration_pattern/mesh_file is ok
    fullpath, name, uri = uriReader(config['calibration_pattern']['mesh_file'])

    return config


def loadYMLConfig(filename):
    """Load configuration from a yml file"""
    try:
        with open(filename, 'r') as f:
            obj = yaml.load(f, Loader=yaml.SafeLoader)
    except OSError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return None

    return obj


def validateLinks(world_link, sensors, urdf):
    try:
        for name, sensor in sensors.items():
            chain = urdf.get_chain(world_link, sensor.link)
            if sensor.parent_link not in chain or sensor.child_link not in chain:
                print("{}: The links '{}' and '{}' are not parte of the same chain.".format(sensor.name,
                                                                                            sensor.parent_link,
                                                                                            sensor.child_link))
                return False
    except KeyError as e:
        link_name = str(e).strip("'")
        if link_name == urdf.get_root():
            print("Configuration contains an unknown base link: {}".format(world_link))
            return False

        print("Configuration contains an unknown link: {}".format(link_name))
        return False

    return True


def laser_scan_msg_to_xy(msg):
    data = message_converter.convert_ros_message_to_dictionary(msg)
    return laser_scan_data_to_xy(data)


def laser_scan_data_to_xy(data):
    ranges = data['ranges']
    angle_min = data['angle_min']
    angle_increment = data['angle_increment']

    x = []
    y = []
    for range_idx, r in enumerate(ranges):
        theta = angle_min + angle_increment * range_idx
        x.append(r * math.cos(theta))
        y.append(r * math.sin(theta))

    return x, y


def getMessageTypeFromTopic(topic):
    # Wait for a message to infer the type
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    rospy.loginfo('Waiting for first message on topic ' + topic + ' ...')
    msg = rospy.wait_for_message(topic, rospy.AnyMsg)

    connection_header = msg._connection_header['type'].split('/')
    # ros_pkg = connection_header[0] + '.msg'
    msg_type_str = connection_header[1]
    msg_type = eval(msg_type_str)

    return msg_type_str, msg_type


def draw_concentric_circles(plt, radii, color='tab:gray'):
    for r in radii:  # meters
        x = []
        y = []
        for theta in np.linspace(0, 2 * math.pi, num=150):
            x.append(r * math.cos(theta))
            y.append(r * math.sin(theta))
        plt.plot(x, y, '-', color='tab:gray')

        x = r * math.cos(math.pi / 2)
        y = r * math.sin(math.pi / 2)
        plt.text(x + .5, y, str(r) + ' (m)', color='tab:gray')


def draw_2d_axes(plt):
    plt.ylabel('y')
    plt.xlabel('x')
    ax = plt.axes()
    ax.arrow(0, 0, 10, 0, head_width=.5, head_length=1, fc='r', ec='r')
    ax.arrow(0, 0, 0, 10, head_width=.5, head_length=1, fc='g', ec='g')

    plt.text(10, 0.5, 'X', color='red')
    plt.text(-1.0, 10, 'Y', color='green')


def printRosTime(time, prefix=""):
    print(prefix + str(time.secs) + "." + str(time.nsecs))


def getMaxTimeDelta(stamps):
    if len(stamps) < 2:  # need at least two time stamps to compute a delta
        return None

    pairs = list(itertools.combinations(stamps, 2))
    max_duration = rospy.Duration(0)
    for p1, p2 in pairs:
        d = abs(p1 - p2)
        if d > max_duration:
            max_duration = d

    return max_duration


def getMaxTime(stamps):
    reference_time = rospy.Time.now()  # get a time at the start of this call
    durations = [abs((stamp - reference_time).to_sec()) for stamp in stamps]

    max_duration = max(durations)
    max_time = reference_time + rospy.Duration(max_duration)

    printRosTime(reference_time, "reference_time: ")
    printRosTime(max_time, "max_time: ")
    print("durations = " + str(durations))
    print("max_duration = " + str(max_duration))

    return max_time


def getAverageTime(stamps):
    reference_time = rospy.Time.now()  # get a time at the start of this call
    durations = [(stamp - reference_time).to_sec() for stamp in stamps]
    avg_duration = sum(durations) / len(durations)
    avg_time = reference_time + rospy.Duration(avg_duration)

    printRosTime(reference_time, "reference_time: ")
    printRosTime(avg_time, "avg_time: ")
    print("durations = " + str(durations))
    print("avg_duration = " + str(avg_duration))

    return avg_time


def colormapToRVizColor(color):
    """ Converts a Matbplotlib colormap into an rviz display color format."""
    return str(int(color[0] * 255)) + '; ' + str(int(color[1] * 255)) + '; ' + str(
        int(color[2] * 255))
