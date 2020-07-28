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
from geometry_msgs.msg import Transform
from urlparse import urlparse
import sensor_msgs.point_cloud2 as pc2
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
        print
        "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if verbose:
                print
                line,
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

    assert (os.path.exists(fullpath)), Fore.RED + fullpath + ' does not exist.'
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


def datatype_to_size_type(datatype):
    """
    Takes a UINT8 datatype field from a PointFields and returns the size in
    bytes and a char for its type ('I': int, 'U': unsigned, 'F': float)
    """
    # might be nicer to look it up in message definition but quicker just to
    # do this.
    if datatype in [2, 3, 4]:
        t = 'U'
    elif datatype in [1, 3, 5]:
        t = 'I'
    elif datatype in [7, 8]:
        t = 'F'
    else:
        raise Exception("Unknown datatype in PointField")

    if datatype < 3:
        s = 1
    elif datatype < 5:
        s = 2
    elif datatype < 8:
        s = 4
    elif datatype < 9:
        s = 8
    else:
        raise Exception("Unknown datatype in PointField")

    return s, t

def size_type_to_datatype(size, type):
    """
    Given a .pcd size/type pair, return a sensor_msgs/PointField datatype
    """
    if type == "F":
        if size == 4:
            return 7
        if size == 8:
            return 8
    if type == "I":
        if size == 1:
            return 1
        if size == 2:
            return 3
        if size == 4:
            return 5
    if type == "U":
        if size == 1:
            return 2
        if size == 2:
            return 4
        if size == 4:
            return 6
    raise Exception("Unknown size/type pair in .pcd")

def write_pcd(filename, pointcloud, overwrite=False, viewpoint=None,
              mode='binary'):
    """
    Writes a sensor_msgs::PointCloud2 to a .pcd file.
    :param filename - the pcd file to write
    :param pointcloud - sensor_msgs::PointCloud2 to write to a file
    :param overwrite - if True, allow overwriting existing files
    :param viewpoint - the camera viewpoint, (x,y,z,qw,qx,qy,qz)
    :param mode - the writing mode: 'ascii' for human readable, 'binary' for
                  a straight dump of the binary data, 'binary_stripped'
                  to strip out data padding before writing (saves space but it slow)
    """
    assert isinstance(pointcloud, PointCloud2)
    if mode not in ['ascii', 'binary', 'binary_stripped']:
        raise Exception("Mode must be 'binary' or 'ascii'")
    if not overwrite and os.path.isfile(filename):
        raise Exception("File exists.")
    try:
        with open(filename, "w") as f:
            f.write("VERSION .7\n")
            _size = {}
            _type = {}
            _count = {}
            _offsets = {}
            _fields = []
            _size['_'] = 1
            _count['_'] = 1
            _type['_'] = 'U'
            offset = 0
            for field in pointcloud.fields:
                if field.offset != offset:
                    # some padding
                    _fields.extend(['_'] * (field.offset - offset))
                isinstance(field, PointField)
                _size[field.name], _type[field.name] = datatype_to_size_type(field.datatype)
                _count[field.name] = field.count
                _offsets[field.name] = field.offset
                _fields.append(field.name)
                offset = field.offset + _size[field.name] * _count[field.name]
            if pointcloud.point_step != offset:
                _fields.extend(['_'] * (pointcloud.point_step - offset))

            if mode != 'binary':
                # remove padding fields
                while True:
                    try:
                        _fields.remove('_')
                    except:
                        break

            # _fields = _count.keys()
            _fields_str = reduce(lambda a, b: a + ' ' + b,
                                 map(lambda x: "{%s}" % x,
                                     _fields))

            f.write("FIELDS ")
            f.write(reduce(lambda a, b: a + ' ' + b,
                           _fields))
            f.write("\n")
            f.write("SIZE ")
            f.write(_fields_str.format(**_size))
            f.write("\n")
            f.write("TYPE ")
            f.write(_fields_str.format(**_type))
            f.write("\n")
            f.write("COUNT ")
            f.write(_fields_str.format(**_count))

            f.write("\n")
            f.write("WIDTH %s" % pointcloud.width)
            f.write("\n")
            f.write("HEIGHT %s" % pointcloud.height)
            f.write("\n")

            if viewpoint is None:
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            else:
                try:
                    assert len(viewpoint) == 7
                except:
                    raise Exception("viewpoint argument must be  tuple "
                                    "(x y z qx qy qz qw)")
                f.write("VIEWPOINT {} {} {} {} {} {} {}\n".format(*viewpoint))

            f.write("POINTS %d\n" % (pointcloud.width * pointcloud.height))
            if mode == "binary":
                # TODO: check for row padding.
                f.write("DATA binary\n")
                f.write(bytearray(pointcloud.data))
            elif mode == "binary_stripped":
                f.write("DATA binary\n")
                if pointcloud.point_step == sum([v[0] * v[1] for v in zip(_size.values(),
                                                                          _count.values())]):  # danger, assumes ordering
                    # ok to just blast it all out; TODO: this assumes row step has no padding
                    f.write(bytearray(pointcloud.data))
                else:
                    # strip all the data padding
                    _field_step = {}
                    for field in _fields:
                        _field_step[field] = _size[field] * _count[field]
                    out = bytearray(sum(_field_step.values()) * pointcloud.width * pointcloud.height)
                    b = 0
                    for v in range(pointcloud.height):
                        offset = pointcloud.row_step * v
                        for u in range(pointcloud.width):
                            for field in _fields:
                                out[b:b + _field_step[field]] = pointcloud.data[offset + _offsets[field]:
                                                                                offset + _offsets[field] + _field_step[
                                                                                    field]]
                                b += _field_step[field]
                            offset += pointcloud.point_step
                    f.write(out)
            else:
                f.write("DATA ascii\n")
                for p in pc2.read_points(pointcloud, _fields):
                    for i, field in enumerate(_fields):
                        f.write("%f " % p[i])
                    f.write("\n")

    except IOError, e:
        raise Exception("Can't write to %s: %s" % (filename, e.message))

def read_pcd(filename, cloud_header=None, get_tf=True):
    if not os.path.isfile(filename):
        raise Exception("[read_pcd] File does not exist.")
    string_array =  lambda x: x.split()
    float_array  =  lambda x: [float(j) for j in x.split()]
    int_array  =  lambda x: [int(j) for j in x.split()]
    word =  lambda x: x.strip()
    headers =  [("VERSION", float),
                ("FIELDS", string_array),
                ("SIZE", int_array),
                ("TYPE", string_array),
                ("COUNT", int_array),
                ("WIDTH", int),
                ("HEIGHT", int),
                ("VIEWPOINT", float_array),
                ("POINTS", int),
                ("DATA", word)]
    header = {}
    with open(filename, "r") as pcdfile:
        while len(headers) > 0:
            line = pcdfile.readline()
            if line == "":
                raise Exception("[read_pcd] EOF reached while looking for headers.")
            f, v = line.split(" ", 1)
            if f.startswith("#"):
                continue
            if f not in zip(*headers)[0]:
                raise Exception("[read_pcd] Field '{}' not known or duplicate.".format(f))
            func =  headers[zip(*headers)[0].index(f)][1]
            header[f] = func(v)
            headers.remove((f, func))
        data =  pcdfile.read()
    # Check the number of points
    if header["VERSION"] != 0.7:
        raise Exception("[read_pcd] only PCD version 0.7 is understood.")
    if header["DATA"] != "binary":
        raise Exception("[read_pcd] Only binary .pcd files are readable.")
    if header["WIDTH"] * header["HEIGHT"] != header["POINTS"]:
        raise Exception("[read_pcd] POINTS count does not equal WIDTH*HEIGHT")

    cloud = PointCloud2()
    cloud.point_step = sum([size * count
                            for size, count in zip(header["SIZE"], header["COUNT"])])
    cloud.height = header["HEIGHT"]
    cloud.width = header["WIDTH"]
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_bigendian = False
    if cloud.row_step * cloud.height > len(data):
        raise Exception("[read_pcd] Data size mismatch.")
    offset = 0
    for field, size, type, count in zip(header["FIELDS"],
                                        header["SIZE"],
                                        header["TYPE"],
                                        header["COUNT"]):

        if field != "_":
            pf =  PointField()
            pf.count = count
            pf.offset = offset
            pf.name = field
            pf.datatype = size_type_to_datatype(size, type)
            cloud.fields.append(pf)
            offset += size * count

    cloud.data = data[0:cloud.row_step * cloud.height]
    if cloud_header is not None:
        cloud.header = header
    else:
        cloud.header.frame_id = "/pcd_cloud"

    if get_tf:
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z =  header["VIEWPOINT"][0:3]
        tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z =  header["VIEWPOINT"][3:]

        return cloud, tf

    return cloud
