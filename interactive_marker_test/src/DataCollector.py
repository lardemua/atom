#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import copy
import json
import os
from colorama import Style, Fore

import numpy as np
import cv2
import tf
from cv_bridge import CvBridge
from interactive_markers.menu_handler import *
# from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from visualization_msgs.msg import *
from tf.listener import TransformListener
from TransformationT import TransformationT
from urdf_parser_py.urdf import URDF


# ------------------------
#      BASE CLASSES      #
# ------------------------

# return Fore.GREEN + self.parent + Style.RESET_ALL + ' to ' + Fore.GREEN + self.child + Style.RESET_ALL + ' (' + self.joint_type + ')'

class DataCollector:

    def __init__(self, world_link):
        self.listener = TransformListener()
        self.sensors = []
        self.world_link = world_link
        self.transforms = {}
        self.data_stamp = 0
        self.bridge = CvBridge()
        rospy.sleep(0.5)

        # Parse robot description from param /robot_description
        xml_robot = URDF.from_parameter_server()

        # Add sensors
        print(Fore.BLUE + 'Sensors:' + Style.RESET_ALL)
        for i, xs in enumerate(xml_robot.sensors):
            self.assertXMLSensorAttributes(xs)  # raises exception if not ok

            # Create a dictionary that describes this sensor
            sensor_dict = {'_name': xs.name, 'parent': xs.parent, 'calibration_parent': xs.calibration_parent,
                           'calibration_child': xs.calibration_child}

            print(xs)

            # Wait for a message to infer the type
            # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
            msg = rospy.wait_for_message(xs.topic, rospy.AnyMsg)
            connection_header =  msg._connection_header['type'].split('/')
            ros_pkg = connection_header[0] + '.msg'
            msg_type = connection_header[1]
            print('Topic ' + xs.topic + ' has type ' + msg_type)
            sensor_dict['topic'] = xs.topic
            sensor_dict['msg_type'] = msg_type

            # If topic contains a message type then get a camera_info message to store along with the sensor data
            if sensor_dict['msg_type'] == 'Image': # if it is an image must get camera_info
                sensor_dict['camera_info_topic'] = os.path.dirname(sensor_dict['topic']) + '/camera_info'
                from sensor_msgs.msg import CameraInfo
                camera_info_msg = rospy.wait_for_message(sensor_dict['camera_info_topic'], CameraInfo)
                from rospy_message_converter import message_converter
                sensor_dict['camera_info'] = message_converter.convert_ros_message_to_dictionary(camera_info_msg)



            print(sensor_dict)

            # Get the kinematic chain form world_link to this sensor's parent link
            chain = self.listener.chain(xs.parent, rospy.Time(), self.world_link, rospy.Time(), self.world_link)
            chain_list = []
            for parent, child in zip(chain[0::], chain[1::]):
                key = self.generateKey(parent, child)
                chain_list.append({'key': key, 'parent': parent, 'child': child})

            sensor_dict['chain'] = chain_list  # Add to sensor dictionary
            self.sensors.append(sensor_dict)

            print(Fore.BLUE + xs.name + Style.RESET_ALL + ':\n' + str(sensor_dict))

    def collectSnapshot(self):

        # Collect transforms (for now collect all transforms even if they are fixed)
        abstract_transforms = self.getAllTransforms()
        transforms_dict = {}  # Initialize an empty dictionary that will store all the transforms for this data-stamp

        for ab in abstract_transforms:  # Update all transformations
            print(ab)
            self.listener.waitForTransform(ab['parent'], ab['child'], rospy.Time(), rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform(ab['parent'], ab['child'], rospy.Time())
            key = self.generateKey(ab['parent'], ab['child'])
            transforms_dict[key] = {'trans': trans, 'quat': quat}

        self.transforms[self.data_stamp] = transforms_dict
        self.data_stamp += 1

        for sensor in self.sensors:
            if sensor['msg_type'] == 'Image':
                msg = rospy.wait_for_message(sensor['topic'], Image)
                from cv_bridge import CvBridge, CvBridgeError
                # cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow('sensor', cv_image)
                cv2.waitKey(0)

                # sensor_dict['camera_info'] = message_converter.convert_ros_message_to_dictionary(camera_info_msg)

        # dictionary = message_converter.convert_ros_message_to_dictionary(message)

        # Save to json file
        D = {'sensors': self.sensors, 'transforms': self.transforms}
        self.createJSONFile('./data_collected.json', D)

    def getAllTransforms(self):

        # Get a list of all transforms to collect
        l = []
        for sensor in self.sensors:
            l.extend(sensor['chain'])

        # https://stackoverflow.com/questions/31792680/how-to-make-values-in-list-of-dictionary-unique
        uniq_l = list(map(dict, frozenset(frozenset(i.items()) for i in l)))
        return uniq_l  # get unique values

    def createJSONFile(self, output_file, D):
        print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
        f = open(output_file, 'w')
        print >> f, json.dumps(D, indent=2, sort_keys=True)
        f.close()
        print("Completed.")

    @staticmethod
    def generateKey(parent, child, suffix=''):
        return parent + '-' + child + suffix

    @staticmethod
    def assertXMLSensorAttributes(xml_sensor):
        # Check if we have all the information needed. Abort if not.
        for attr in ['parent', 'calibration_parent', 'calibration_child', 'topic']:
            if not hasattr(xml_sensor, attr):
                raise ValueError(
                    'Element ' + attr + ' for sensor ' + xml_sensor.name + ' must be specified in the urdf/xacro.')
