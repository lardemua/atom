import itertools
import math
import numpy as np

import json
import jsonschema

import rospy
from rospy_message_converter import message_converter

from sensor_msgs.msg import *
from json_minify import json_minify


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
