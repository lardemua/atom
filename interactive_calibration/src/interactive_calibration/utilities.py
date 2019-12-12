
import math
import numpy as np

import json
import jsonschema

import rospy
from rospy_message_converter import message_converter

from sensor_msgs.msg import *

def loadJSONConfig(filename):
    """Load configuration from a json file"""
    try:
        with open(filename, 'r') as f:
            obj = json.load(f)

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
                "required": ["pattern_type", "dimension", "size", "border_size"],
                "additionalProperties": False,
                "properties": {
                    "pattern_type": {"type": "string"},
                    "dimension": {
                        "type": "array",
                        "minItems": 2,
                        "maxItems": 2,
                        "items": {"type": "number"}
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
                print("{}: The links '{}' and '{}' are not parte of the same chain.".format(sensor.name, sensor.parent_link, sensor.child_link))
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
