import itertools
import math
import numpy as np

import json
import os
import rospkg
import subprocess
from colorama import Fore

import yaml
import jsonschema
import rospy
from rospy_message_converter import message_converter
from sensor_msgs.msg import *
from json_minify import json_minify
from urlparse import urlparse


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


def uriReader(resource):
    uri = urlparse(str(resource))
    # print(uri)
    if uri.scheme == 'package':  # using a ros package uri
        # print('This is a ros package')
        rospack = rospkg.RosPack()
        assert (rospack.get_path(uri.netloc)), 'Package ' + uri.netloc + ' does not exist.'
        fullpath = resolvePath(rospack.get_path(uri.netloc) + uri.path)
    elif uri.scheme == 'file':  # local file
        # print('This is a local file')
        fullpath = resolvePath(uri.netloc + uri.path)
    elif uri.scheme == '':  # no scheme, assume local file
        # print('This is a local file')
        fullpath = resolvePath(uri.netloc + uri.path)
    else:
        raise ValueError('Cannot parse resource "' + resource + '", unknown scheme "' + uri.scheme + '".')

    assert (os.path.exists(fullpath)), Fore.RED + fullpath + ' does not exist. Check your config.yml description file'
    return fullpath, os.path.basename(fullpath), uri


def loadConfig(filename, check_paths=True):
    config = loadYMLConfig(filename)

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


def loadJSONConfig(filename):
    """Load configuration from a json file"""
    try:
        with open(filename, 'r') as f:
            obj = json.loads(json_minify(f.read()))

        _validateJSONConfig(obj)
    except OSError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return None
    except jsonschema.exceptions.ValidationError as e:
        print("Invalid calibration JSON: {} @ /{}".format(e.message, "/".join([str(x) for x in e.path])))
        return None

    return obj


def _validateJSONConfig(obj):
    # Tedious work!!!!
    schema = {
        "type": "object",
        "required": ["sensors", "world_link", "calibration_pattern"],
        "properties": {
            "sensors": {
                "type": "object",
                "patternProperties": {
                    "^(.*)$": {
                        "type": "object",
                        "required": ["link", "parent_link", "child_link"],
                        "additionalProperties": False,
                        "properties": {
                            "link": {"type": "string"},
                            "parent_link": {"type": "string"},
                            "child_link": {"type": "string"},
                            "topic_name": {"type": "string"},
                        }
                    }
                }
            },
            "anchored_sensor": {"type": "string"},
            "world_link": {"type": "string"},
            "max_duration_between_msgs": {"type": "number"},
            "calibration_pattern": {
                "type": "object",
                "required": ["link", "parent_link", "pattern_type", "dimension", "size", "border_size"],
                "additionalProperties": False,
                "properties": {
                    "link": {"type": "string"},
                    "parent_link": {"type": "string"},
                    "origin": {
                        "type": "array",
                        "minItems": 6,
                        "maxItems": 6,
                        "items": {"type": "number"}
                    },
                    "fixed": {"type": "boolean"},
                    "pattern_type": {"type": "string"},
                    "dictionary": {"type": "string"},
                    "dimension": {
                        "type": "object",
                        "required": ["x", "y"],
                        "additionalProperties": False,
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                        }
                    },
                    "size": {"type": "number"},
                    "inner_size": {"type": "number"},
                    "border_size": {"type": "number"}
                }
            }
        }
    }

    jsonschema.validate(obj, schema)


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
    durations = [(stamp - reference_time).to_sec() for stamp in stamps]

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
