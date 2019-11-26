
import copy
import os
import shutil
import cv2
import numpy as np

from cv_bridge import CvBridge
from colorama import Style, Fore

from interactive_markers.menu_handler import *
from rospy_message_converter import message_converter

import tf
from tf.listener import TransformListener
from visualization_msgs.msg import *
from sensor_msgs.msg        import *

from transformation_t import TransformationT
from interactive_calibration.utilities                import CalibConfig
from interactive_calibration.interactive_data_labeler import InteractiveDataLabeler

class DataCollectorAndLabeler:

    def __init__(self, output_folder, server, menu_handler, marker_size, calibration_file):

        if not os.path.exists(output_folder):
            os.mkdir(output_folder)  # Create the new folder
        else:
            while True:
                msg = Fore.YELLOW + "To continue, the directory '{}' will be delete.\n"
                msg = msg         + "Do you wish to continue? [y/N] " + Style.RESET_ALL

                answer = raw_input(msg.format(output_folder))
                if len(answer) > 0 and answer[0].lower() in ('y', 'n'):
                    if answer[0].lower() == 'n': sys.exit(1)
                    else: break
                else: sys.exit(1) # defaults to N

            shutil.rmtree(output_folder)  # Delete old folder
            os.mkdir(output_folder)       # Recreate the folder

        self.output_folder = output_folder
        self.listener = TransformListener()
        self.sensors = {}
        self.transforms = {}
        self.sensor_labelers = {}
        self.server = server
        self.menu_handler = menu_handler

        self.data_stamp = 0

        # self.transforms = {}
        # self.data = []
        self.collections = {}

        self.bridge = CvBridge()

        # wait for transformations
        rospy.sleep(0.5)

        config = CalibConfig()
        ok = config.loadJSON(calibration_file)
        if not ok: sys.exit(1) # loadJSON should tell you why.

        self.world_link = config.world_link

        # Add sensors
        print(Fore.BLUE + 'Sensors:' + Style.RESET_ALL)

        print('Number of sensors: ' + str(len(config.sensors)))

        # Go through the sensors in the calib config.
        for sensor_key, value in config.sensors.items():
            print('visiting sensor ' + sensor_key)
            # continue
            # TODO put this in a function and adapt for the json case

            # Create a dictionary that describes this sensor
            sensor_dict = {'_name': sensor_key, 'parent': value.link,
                           'calibration_parent': value.parent_link,
                           'calibration_child':  value.child_link}

            #TODO replace by utils function
            print("Waiting for message")
            msg = rospy.wait_for_message(value.topic_name, rospy.AnyMsg)
            connection_header = msg._connection_header['type'].split('/')
            ros_pkg = connection_header[0] + '.msg'
            msg_type = connection_header[1]
            print('Topic ' + value.topic_name + ' has type ' + msg_type)
            sensor_dict['topic'] = value.topic_name
            sensor_dict['msg_type'] = msg_type

            # If topic contains a message type then get a camera_info message to store along with the sensor data
            if sensor_dict['msg_type'] == 'Image':  # if it is an image must get camera_info
                sensor_dict['camera_info_topic'] = os.path.dirname(sensor_dict['topic']) + '/camera_info'
                from sensor_msgs.msg import CameraInfo
                camera_info_msg = rospy.wait_for_message(sensor_dict['camera_info_topic'], CameraInfo)
                from rospy_message_converter import message_converter
                sensor_dict['camera_info'] = message_converter.convert_ros_message_to_dictionary(camera_info_msg)

            # Get the kinematic chain form world_link to this sensor's parent link
            chain = self.listener.chain(value.link, rospy.Time(), self.world_link, rospy.Time(), self.world_link)
            chain_list = []
            for parent, child in zip(chain[0::], chain[1::]):
                key = self.generateKey(parent, child)
                chain_list.append({'key': key, 'parent': parent, 'child': child})

            sensor_dict['chain'] = chain_list  # Add to sensor dictionary
            self.sensors[sensor_key] = sensor_dict

            sensor_labeler = InteractiveDataLabeler(self.server, self.menu_handler, sensor_dict, marker_size, config.pattern.dimension[0], config.pattern.dimension[1])

            self.sensor_labelers[sensor_key] = sensor_labeler

            print('finished visiting sensor ' + sensor_key)
            print(Fore.BLUE + sensor_key + Style.RESET_ALL + ':\n' + str(sensor_dict))

        print('sensor_labelers:')
        print(self.sensor_labelers)

        self.abstract_transforms = self.getAllTransforms()

        # --------------------------------------
        # Collect transforms (for now collect all transforms even if they are fixed)
        # --------------------------------------
        transforms_dict = {}  # Initialize an empty dictionary that will store all the transforms for this data-stamp

        for ab in self.abstract_transforms:  # Update all transformations
            print(ab)
            self.listener.waitForTransform(ab['parent'], ab['child'], rospy.Time(), rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform(ab['parent'], ab['child'], rospy.Time())
            key = self.generateKey(ab['parent'], ab['child'])
            transforms_dict[key] = {'trans': trans, 'quat': quat}

        # self.transforms[self.data_stamp] = transforms_dict
        self.transforms = transforms_dict

    def collectSnapshot(self):

        # --------------------------------------
        # Collect sensor data (images, laser scans, etc)
        # --------------------------------------
        all_sensor_data_dict = {}
        for sensor_name, sensor in self.sensors.iteritems():

            print('collectSnapshot: sensor_name ' + sensor_name)

            # TODO add exception also for point cloud and depht image
            if sensor['msg_type'] == 'Image':  #
                # Get latest ros message on this topic
                print("waiting for message " + sensor['topic'])
                msg = rospy.wait_for_message(sensor['topic'], Image)
                print("received message " + sensor['topic'])

                # Convert to opencv image and save image to disk
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                filename = self.output_folder + '/' + sensor['_name'] + '_' + str(self.data_stamp) + '.jpg'
                filename_relative = sensor['_name'] + '_' + str(self.data_stamp) + '.jpg'
                print('Data ' + str(self.data_stamp) + ' from sensor ' + sensor['_name'] + ': saving image ' + filename)
                cv2.imwrite(filename, cv_image)
                # cv2.imshow('sensor', cv_image)
                # cv2.waitKey(0)

                # Convert the image to python dictionary
                image_dict = message_converter.convert_ros_message_to_dictionary(msg)

                # Remove data field (which contains the image), and replace by "data_file" field which contains the
                # full path to where the image was saved
                del image_dict['data']
                image_dict['data_file'] = filename_relative

                # Update the data dictionary for this data stamp
                all_sensor_data_dict[sensor['_name']] = image_dict

            else:
                # Get latest ros message on this topic
                # msg = rospy.wait_for_message(sensor['topic'], LaserScan)
                print("waiting for message " + sensor['topic'])
                msg = rospy.wait_for_message(sensor['topic'], eval(sensor['msg_type']))
                print("received message " + sensor['topic'])

                # Update the data dictionary for this data stamp
                all_sensor_data_dict[sensor['_name']] = message_converter.convert_ros_message_to_dictionary(msg)

        # self.data[self.data_stamp] = all_sensor_data_dict
        # self.data.append(all_sensor_data_dict)

        # --------------------------------------
        # Collect sensor labels
        # --------------------------------------
        all_sensor_labels_dict = {}
        for sensor_name, sensor in self.sensors.iteritems():

            if sensor['msg_type'] in ['Image', 'LaserScan']:
                self.sensor_labelers[sensor_name].lock.acquire()
                all_sensor_labels_dict[sensor['_name']] = copy.deepcopy(self.sensor_labelers[sensor['_name']].labels)
                self.sensor_labelers[sensor_name].lock.release()
                # TODO check if more deepcopys are needed
            else:
                #TODO put here a raise error
                pass

        print('----------------\nstarts here\n----------------')
        print(all_sensor_labels_dict)
        print('----------------\nends here\n----------------')
        # --------------------------------------
        # Add a new collection
        # --------------------------------------
        self.collections[self.data_stamp] = {'data': all_sensor_data_dict, 'labels': all_sensor_labels_dict, 'transforms': self.transforms}
        self.data_stamp += 1

        # Save to json file
        D = {'sensors': self.sensors, 'collections': self.collections}
        self.createJSONFile(self.output_folder + '/data_collected.json', D)

    def getAllTransforms(self):

        # Get a list of all transforms to collect
        transforms_list = []
        for sensor_name, sensor in self.sensors.iteritems():
            for transform in sensor['chain']:
                transforms_list.append(transform)

        # https://stackoverflow.com/questions/31792680/how-to-make-values-in-list-of-dictionary-unique
        uniq_l = list(map(dict, frozenset(frozenset(i.items()) for i in transforms_list)))
        return uniq_l  # get unique values

    def createJSONFile(self, output_file, D):
        print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
        f = open(output_file, 'w')
        json.encoder.FLOAT_REPR = lambda f: ("%.4f" % f)  # to get only four decimal places on the json file
        print >> f, json.dumps(D, indent=2, sort_keys=True)
        f.close()
        print("Completed.")

    @staticmethod
    def generateKey(parent, child, suffix=''):
        return parent + '-' + child + suffix

