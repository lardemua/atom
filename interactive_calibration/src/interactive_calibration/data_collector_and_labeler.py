import copy
import json
import os
import shutil
import cv2
import itertools

from cv_bridge import CvBridge
from colorama import Style, Fore
from interactive_markers.menu_handler import *
from rospy_message_converter import message_converter
from tf.listener import TransformListener
from sensor_msgs.msg import *
from interactive_calibration.utilities import CalibConfig
from interactive_calibration.interactive_data_labeler import InteractiveDataLabeler


class DataCollectorAndLabeler:

    def __init__(self, output_folder, server, menu_handler, marker_size, calibration_file):

        if not os.path.exists(output_folder):
            os.mkdir(output_folder)  # Create the new folder
        else:
            while True:
                msg = Fore.YELLOW + "To continue, the directory '{}' will be delete.\n"
                msg = msg + "Do you wish to continue? [y/N] " + Style.RESET_ALL

                answer = raw_input(msg.format(output_folder))
                if len(answer) > 0 and answer[0].lower() in ('y', 'n'):
                    if answer[0].lower() == 'n':
                        sys.exit(1)
                    else:
                        break
                else:
                    sys.exit(1)  # defaults to N

            shutil.rmtree(output_folder)  # Delete old folder
            os.mkdir(output_folder)  # Recreate the folder

        self.output_folder = output_folder
        self.listener = TransformListener()
        self.sensors = {}
        self.sensor_labelers = {}
        self.server = server
        self.menu_handler = menu_handler
        self.data_stamp = 0
        self.collections = {}
        self.bridge = CvBridge()

        self.config = CalibConfig()
        ok = self.config.loadJSON(calibration_file)
        if not ok:
            sys.exit(1)  # loadJSON should tell you why.

        self.world_link = self.config.world_link

        # Add sensors
        print(Fore.BLUE + 'Sensors:' + Style.RESET_ALL)
        print('Number of sensors: ' + str(len(self.config.sensors)))

        # Go through the sensors in the calib config.
        for sensor_key, value in self.config.sensors.items():
            # continue
            # TODO put this in a function and adapt for the json case

            # Create a dictionary that describes this sensor
            sensor_dict = {'_name': sensor_key, 'parent': value.link,
                           'calibration_parent': value.parent_link,
                           'calibration_child': value.child_link}

            # TODO replace by utils function
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

            sensor_labeler = InteractiveDataLabeler(self.server, self.menu_handler, sensor_dict, marker_size,
                                                    self.config.pattern.dimension[0], self.config.pattern.dimension[1])

            self.sensor_labelers[sensor_key] = sensor_labeler

            print('finished visiting sensor ' + sensor_key)
            print(Fore.BLUE + sensor_key + Style.RESET_ALL + ':\n' + str(sensor_dict))

        print('sensor_labelers:')
        print(self.sensor_labelers)

        self.abstract_transforms = self.getAllAbstractTransforms()
        print("abstract_transforms = " + str(self.abstract_transforms))

    def getTransforms(self, abstract_transforms):
        transforms_dict = {}  # Initialize an empty dictionary that will store all the transforms for this data-stamp
        now = rospy.Time.now()

        for ab in abstract_transforms:  # Update all transformations
            self.listener.waitForTransform(ab['parent'], ab['child'], now, rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform(ab['parent'], ab['child'], now)
            key = self.generateKey(ab['parent'], ab['child'])
            transforms_dict[key] = {'trans': trans, 'quat': quat, 'parent': ab['parent'], 'child': ab['child']}

        return transforms_dict

    def collectSnapshot(self):

        # --------------------------------------
        # Collect sensor data and labels (images, laser scans, etc)
        # --------------------------------------
        all_sensor_data_dict = {}
        all_sensor_labels_dict = {}

        # Collect all the transforms
        transforms = self.getTransforms(self.abstract_transforms)

        for sensor_name, sensor in self.sensors.iteritems():
            print('collectSnapshot: sensor_name ' + sensor_name)

            # Lock the semaphore and make a copy of both the data and the labels.
            self.sensor_labelers[sensor_name].lock.acquire()
            msg = copy.deepcopy(self.sensor_labelers[sensor_name].msg)
            labels = copy.deepcopy(self.sensor_labelers[sensor_name].labels)
            self.sensor_labelers[sensor_name].lock.release()

            # TODO add exception also for point cloud and depht image
            # Update sensor data ---------------------------------------------
            if sensor['msg_type'] == 'Image':  # Special case of requires saving image data as png separate files
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert to opencv image and save image to disk
                filename = self.output_folder + '/' + sensor['_name'] + '_' + str(self.data_stamp) + '.jpg'
                filename_relative = sensor['_name'] + '_' + str(self.data_stamp) + '.jpg'
                print('Data ' + str(self.data_stamp) + ' from sensor ' + sensor['_name'] + ': saving image ' + filename)
                cv2.imwrite(filename, cv_image)

                # Convert the sensor data to python dictionary
                image_dict = message_converter.convert_ros_message_to_dictionary(msg)
                del image_dict['data']  # Remove data field (which contains the image), and replace by "data_file"
                # field which contains the  full path to where the image was saved
                image_dict['data_file'] = filename_relative

                # Update the data dictionary for this data stamp
                all_sensor_data_dict[sensor['_name']] = image_dict

            else:
                # Update the data dictionary for this data stamp
                all_sensor_data_dict[sensor['_name']] = message_converter.convert_ros_message_to_dictionary(msg)

            # Update sensor labels ---------------------------------------------
            if sensor['msg_type'] in ['Image', 'LaserScan']:
                all_sensor_labels_dict[sensor_name] = labels
            else:
                raise ValueError('Unknown message type.')

        collection_dict = {'data': all_sensor_data_dict, 'labels': all_sensor_labels_dict, 'transforms': transforms}

        # Check if the message time stamps are "close enough"
        stamps = []  # a list of the several time stamps of the stored messages
        t = rospy.Time()
        for sensor_key, data in collection_dict['data'].items():
            t.set(data['header']['stamp']['secs'], data['header']['stamp']['nsecs'])
            stamps.append(copy.deepcopy(t))

        pairs = list(itertools.combinations(stamps, 2))
        max_duration = rospy.Duration(0)
        for p1, p2 in pairs:
            d = abs(p1 - p2)
            if d > max_duration:
                max_duration = d

        if max_duration.to_sec() > float(self.config.obj['max_duration_between_msgs']):
            rospy.logerr('Max duration between msgs in collection is ' + str(max_duration.to_sec())
                         + ' . Not saving collection.')
            return None
        else:
            rospy.loginfo('Max duration between msgs in collection is ' + str(max_duration.to_sec()))

        # Build a collection dictionary
        self.collections[self.data_stamp] = collection_dict
        self.data_stamp += 1

        # Save to json file
        D = {'sensors': self.sensors, 'collections': self.collections, 'calibration_config': self.config.obj}
        self.createJSONFile(self.output_folder + '/data_collected.json', D)

    def getAllAbstractTransforms(self):

        rospy.sleep(0.5)  # wait for transformations
        # Get a list of all transforms to collect
        transforms_list = []

        now = rospy.Time.now()
        all_frames = self.listener.getFrameStrings()

        for frame in all_frames:
            chain = self.listener.chain(frame, now, self.world_link, now, self.world_link)
            for idx in range(0, len(chain) - 1):
                parent = chain[idx]
                child = chain[idx + 1]
                transforms_list.append({'parent': parent, 'child': child, 'key': self.generateKey(parent, child)})

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
