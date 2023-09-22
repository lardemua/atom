# stdlib
import copy
import json
import os
import time
import getpass
from datetime import datetime, date

import numpy as np
import tf2_ros
import yaml
import atom_core.config_io
import atom_core.dataset_io

# 3rd-party
import atom_msgs.srv
import tf
from matplotlib import cm
from cv_bridge import CvBridge
from colorama import Style, Fore
from interactive_markers.menu_handler import *
from rospy_message_converter import message_converter
from tf.listener import TransformListener
from sensor_msgs.msg import *
from urdf_parser_py.urdf import URDF

# local packages
from atom_core.naming import generateKey
from atom_core.ros_utils import printRosTime, getMaxTimeDelta, getMaxTime, getAverageTime
from atom_core.config_io import execute, loadConfig
from atom_calibration.collect.interactive_data_labeler import InteractiveDataLabeler


class DataCollectorAndLabeler:

    def __init__(self, args, server, menu_handler):

        self.output_folder = atom_core.config_io.resolvePath(args['output_folder'])

        if os.path.exists(self.output_folder) and not args['overwrite']:  # dataset path exists, abort
            print('\n' + Fore.RED + 'Error: Dataset ' + self.output_folder +
                  ' exists.\nIf you want to replace it add a "--overwrite" flag.' + Style.RESET_ALL + '\n')
            rospy.signal_shutdown()

        elif os.path.exists(self.output_folder) and args['overwrite']:  # move existing path to a backup location
            now = datetime.now()
            dt_string = now.strftime("%Y-%m-%d-%H-%M-%S")
            basename = os.path.basename(self.output_folder)
            backup_folder = '/tmp/' + basename + '_' + dt_string

            time.sleep(2)
            print('\n\nWarning: Dataset ' + Fore.YELLOW + self.output_folder + Style.RESET_ALL +
                  ' exists.\nMoving it to a new folder: ' + Fore.YELLOW + backup_folder +
                  '\nThis will be deleted after a system reboot!' + Style.RESET_ALL + '\n\n')
            time.sleep(2)

            execute('mv ' + self.output_folder + ' ' + backup_folder, verbose=True)

        os.mkdir(self.output_folder)  # Recreate the folder

        self.listener = TransformListener()
        self.sensors = {}
        self.sensor_labelers = {}
        self.server = server
        self.menu_handler = menu_handler
        self.data_stamp = 0
        self.timestamp = ""
        self.date = ""
        self.user = ""
        self.collections = {}
        self.additional_data = {}
        self.metadata = {}
        self.bridge = CvBridge()
        self.dataset_version = "2.0"

        # print(args['calibration_file'])
        self.config = loadConfig(args['calibration_file'])
        if self.config is None:
            sys.exit(1)  # loadJSON should tell you why.

        self.world_link = self.config['world_link']

        # Create a colormap so that we have one color per sensor
        self.cm_sensors = cm.Pastel2(np.linspace(0, 1, len(self.config['sensors'].keys())))

        # Add sensors
        print(Fore.BLUE + 'Sensors:' + Style.RESET_ALL)
        print('Number of sensors: ' + str(len(self.config['sensors'])))

        # Go through the sensors in the calib config.
        sensor_idx = 0
        for sensor_key, value in self.config['sensors'].items():

            # Create a dictionary that describes this sensor
            sensor_dict = {'_name': sensor_key, 'modality': value['modality'], 'parent': value['link'],
                           'calibration_parent': value['parent_link'],
                           'calibration_child': value['child_link']}

            # TODO replace by utils function
            print("Waiting for message " + value['topic_name'] + ' ...')
            msg = rospy.wait_for_message(value['topic_name'], rospy.AnyMsg)
            print('... received!')
            connection_header = msg._connection_header['type'].split('/')
            ros_pkg = connection_header[0] + '.msg'
            msg_type = connection_header[1]
            print('Topic ' + value['topic_name'] + ' has type ' + msg_type)
            sensor_dict['topic'] = value['topic_name']
            sensor_dict['msg_type'] = msg_type
            modality = value['modality']

            # If topic contains a message type then get a camera_info message to store along with the sensor data
            if modality == 'rgb' or modality == 'depth':  # if it is an image must get camera_info
                sensor_dict['camera_info_topic'] = os.path.dirname(sensor_dict['topic']) + '/camera_info'
                from sensor_msgs.msg import CameraInfo
                print('Waiting for camera_info message on topic ' + sensor_dict['camera_info_topic'] + ' ...')
                camera_info_msg = rospy.wait_for_message(sensor_dict['camera_info_topic'], CameraInfo)
                print('... received!')
                from rospy_message_converter import message_converter
                sensor_dict['camera_info'] = message_converter.convert_ros_message_to_dictionary(camera_info_msg)
                # sensor_dict['camera_info_msg'] = camera_info_msg
                # print(camera_info_msg)

            # Get the kinematic chain form world_link to this sensor's parent link
            now = rospy.Time()
            print('Waiting for transformation from ' + value['link'] + ' to ' + self.world_link)
            self.listener.waitForTransform(value['link'], self.world_link, now, rospy.Duration(5))
            print('... received!')
            chain = self.listener.chain(value['link'], now, self.world_link, now, self.world_link)

            chain_list = []
            for parent, child in zip(chain[0::], chain[1::]):
                key = generateKey(parent, child)
                chain_list.append({'key': key, 'parent': parent, 'child': child})

            sensor_dict['chain'] = chain_list  # Add to sensor dictionary
            self.sensors[sensor_key] = sensor_dict

            print('config = ' + str(self.config))

            label_data = True
            if not args['skip_sensor_labeling'] is None:
                if args['skip_sensor_labeling'](sensor_key):  # use the lambda expression csf
                    label_data = False
            sensor_labeler = InteractiveDataLabeler(self.server, self.menu_handler, sensor_dict,
                                                    args['marker_size'], self.config['calibration_pattern'],
                                                    color=tuple(self.cm_sensors[sensor_idx, :]), label_data=label_data)

            self.sensor_labelers[sensor_key] = sensor_labeler

            print('Setup for sensor ' + sensor_key + ' is complete.')
            print(Fore.BLUE + sensor_key + Style.RESET_ALL + ':\n' + str(sensor_dict))
            sensor_idx += 1

        # Additional data loop
        if 'additional_data' in self.config:
            for description, value in self.config['additional_data'].items():
                data_dict = {'_name': description, 'modality': value['modality'], 'parent': value['link'],
                             'calibration_parent': value['parent_link'], 'calibration_child': value['child_link']}

                print("Waiting for message " + value['topic_name'] + ' ...')
                msg = rospy.wait_for_message(value['topic_name'], rospy.AnyMsg)
                print('... received!')
                connection_header = msg._connection_header['type'].split('/')
                msg_type = connection_header[1]
                print('Topic ' + value['topic_name'] + ' has type ' + msg_type)
                data_dict['topic'] = value['topic_name']
                data_dict['msg_type'] = msg_type

                sensor_labeler = InteractiveDataLabeler(self.server, self.menu_handler, data_dict,
                                                        args['marker_size'], self.config['calibration_pattern'],
                                                        label_data=False)

                self.sensor_labelers[description] = sensor_labeler
                self.additional_data[description] = data_dict


        self.abstract_transforms = self.getAllAbstractTransforms()
        # print("abstract_transforms = " + str(self.abstract_transforms))

        # Add service to make the dataset json available
        self.service_get_dataset = rospy.Service('~get_dataset',
                                                 atom_msgs.srv.GetDataset,
                                                 self.callbackGetDataset)
        # Add service to save a new collection
        self.service_save_collection = rospy.Service('~save_collection',
                                                     atom_msgs.srv.SaveCollection,
                                                     self.callbackSaveCollection)

        # Add service to delete a new collection
        self.service_delete_collection = rospy.Service('~delete_collection',
                                                       atom_msgs.srv.DeleteCollection,
                                                       self.callbackDeleteCollection)

    def callbackDeleteCollection(self, request):
        print('callbackDeleteCollection service called')

        try:  # make sure we can convert the collection_name to int
            collection_name_int = int(request.collection_name)
        except ValueError as verr:  # do job to handle: s does not contain anything convertible to int
            response = atom_msgs.srv.DeleteCollectionResponse()
            response.success = False
            response.message = 'Failure. Cannot convert collection ' + request.collection_name + ' to string.'
            return response
        except Exception as ex:  # do job to handle: Exception occurred while converting to int
            response = atom_msgs.srv.DeleteCollectionResponse()
            response.success = False
            response.message = 'Failure. Cannot convert collection ' + request.collection_name + ' to string.'

        # Lock the semaphore for all labelers
        self.lockAllLabelers()
        response = atom_msgs.srv.DeleteCollectionResponse()
        if collection_name_int in self.collections.keys():
            del self.collections[collection_name_int]
            response.success = True
            response.message = 'Collection ' + request.collection_name + ' deleted'

            # Save new dataset to json file
            D = {'sensors': self.sensors, 'additional_sensor_data': self.additional_data,
                 'collections': self.collections, 'calibration_config': self.config}
            output_file = self.output_folder + '/dataset.json'
            atom_core.dataset_io.saveResultsJSON(output_file, D)

            print(Fore.YELLOW + 'Deleted collection ' + request.collection_name + Style.RESET_ALL)
        else:
            print('Failure. Collection ' + request.collection_name + ' does not exist.')
            response.success = False
            response.message = 'Failure. Collection ' + request.collection_name + ' does not exist.'

        print(self.collections.keys())
        self.unlockAllLabelers()

        return response

    def callbackSaveCollection(self, request):
        print('callbackSaveCollection service called')

        # Call internal save collection
        self.saveCollection()

        response = atom_msgs.srv.SaveCollectionResponse()
        response.success = True
        response.message = "Collection saved"
        return response

    def callbackGetDataset(self, request):
        print('callbackGetDataset service called')

        dataset_file = self.output_folder + '/dataset.json'
        try:
            file = open(dataset_file, 'r')
            dataset_stream = file.read()
            success = True
        except:
            dataset_stream = ''
            success = False

        response = atom_msgs.srv.GetDatasetResponse()
        response.dataset_json = dataset_stream
        response.success = success
        return response

    def getTransforms(self, abstract_transforms, time=None):
        transforms_dict = {}  # Initialize an empty dictionary that will store all the transforms for this data-stamp

        if time is None:
            time = rospy.Time.now()

        for ab in abstract_transforms:  # Update all transformations
            self.listener.waitForTransform(ab['parent'], ab['child'], time, rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform(ab['parent'], ab['child'], time)
            key = generateKey(ab['parent'], ab['child'])
            transforms_dict[key] = {'trans': trans, 'quat': quat, 'parent': ab['parent'], 'child': ab['child']}

        return transforms_dict

    def lockAllLabelers(self):
        for sensor_name, sensor in self.sensors.items():
            self.sensor_labelers[sensor_name].lock.acquire()
        print("Locked all labelers")

    def unlockAllLabelers(self):
        for sensor_name, sensor in self.sensors.items():
            self.sensor_labelers[sensor_name].lock.release()
        print("Unlocked all labelers")

    def getLabelersTimeStatistics(self):
        stamps = []  # a list of the several time stamps of the stored messages
        for sensor_name, sensor in self.sensors.items():
            stamps.append(copy.deepcopy(self.sensor_labelers[sensor_name].msg.header.stamp))

        max_delta = getMaxTimeDelta(stamps)
        # TODO : this is because of Andre's bag file problem. We should go back to the getAverageTime
        average_time = getAverageTime(stamps)  # For looking up transforms use average time of all sensor msgs
        # average_time = getMaxTime(stamps)  # For looking up transforms use average time of all sensor msgs

        print('Times:')
        for stamp, sensor_name in zip(stamps, self.sensors):
            printRosTime(stamp, prefix=sensor_name + ': ')

        return stamps, average_time, max_delta

    def saveCollection(self):

        # --------------------------------------
        # collect sensor data and labels (images, laser scans, etc)
        # --------------------------------------

        # Lock the semaphore for all labelers
        self.lockAllLabelers()

        # Analyze message time stamps and decide if collection can be stored
        stamps, average_time, max_delta = self.getLabelersTimeStatistics()

        if max_delta is not None:  # if max_delta is None (only one sensor), continue
            if max_delta.to_sec() > float(self.config['max_duration_between_msgs']):  # times are close enough?
                rospy.logwarn('Max duration between msgs in collection is ' + str(max_delta.to_sec()) +
                              '. Not saving collection.')
                self.unlockAllLabelers()
                return None
            else:  # test passed
                rospy.loginfo('Max duration between msgs in collection is ' + str(max_delta.to_sec()))

        # collect all the transforms
        print('average_time=' + str(average_time))
        transforms = self.getTransforms(self.abstract_transforms, average_time)  # use average time of sensor msgs
        printRosTime(average_time, "Collected transforms for time ")

        all_sensor_data_dict = {}
        all_sensor_labels_dict = {}
        all_additional_data_dict = {}
        # metadata={}

        for sensor_key, sensor in self.sensors.items():
            print('Collecting data from ' + Fore.BLUE + sensor_key + Style.RESET_ALL + ': sensor_key')

            msg = copy.deepcopy(self.sensor_labelers[sensor_key].msg)
            labels = copy.deepcopy(self.sensor_labelers[sensor_key].labels)

            # Update the data dictionary for this data stamp
            all_sensor_data_dict[sensor['_name']] = message_converter.convert_ros_message_to_dictionary(msg)

            # Update sensor labels ---------------------------------------------
            # if sensor['msg_type'] in ['Image', 'LaserScan', 'PointCloud2']:
            #     all_sensor_labels_dict[sensor_key] = labels
            if sensor['modality'] in ['rgb', 'lidar2d', 'depth', 'lidar3d']:
                all_sensor_labels_dict[sensor_key] = labels
            else:
                raise ValueError('Unknown message type.')

        for description, sensor in self.additional_data.items():
            msg = copy.deepcopy(self.sensor_labelers[description].msg)
            all_additional_data_dict[sensor['_name']] = message_converter.convert_ros_message_to_dictionary(msg)

        collection_dict = {'data': all_sensor_data_dict, 'labels': all_sensor_labels_dict, 'transforms': transforms,
                           'additional_data': all_additional_data_dict}
        collection_key = str(self.data_stamp).zfill(3)
        self.collections[collection_key] = collection_dict
        self.data_stamp += 1

        dataset_name = self.output_folder.split('/')[-1]
        description_file, _, _ = atom_core.config_io.uriReader(self.config['description_file'])
        description = URDF.from_xml_file(description_file)

        # Create metadata.
        self.metadata = {"timestamp": str(time.time()), "date": time.ctime(time.time()), "user": getpass.getuser(),
                         'version': self.dataset_version, 'robot_name': description.name,
                         'dataset_name': dataset_name}

        # Save to json file.
        D = {'sensors': self.sensors, 'additional_sensor_data': self.additional_data, 'collections': self.collections,
             'calibration_config': self.config, '_metadata': self.metadata}
        output_file = self.output_folder + '/dataset.json'
        atom_core.dataset_io.saveResultsJSON(output_file, D)

        self.unlockAllLabelers()

    def getAllAbstractTransforms(self):

        # Get a list of all transforms to collect
        transforms_list = []

        rospy.sleep(0.5)
        now = rospy.Time.now()

        # Error came up in 
        # https://github.com/miguelriemoliveira/agri-gaia-test/issues/1
        # Identified that this was deprecated in 
        # https://answers.ros.org/question/335327/yamlload-is-deprecated-when-source-melodic_wssetupbash-is-executed/
        # Using solution from 
        # https://answers.ros.org/question/347857/how-to-get-list-of-all-tf-frames-programatically/
        # all_frames = self.listener.getFrameStrings()
        frames_dict = yaml.safe_load(self.listener._buffer.all_frames_as_yaml())
        all_frames = list(frames_dict.keys())

        for frame in all_frames:
            print('Waiting for transformation from ' + frame + ' to ' + self.world_link + '(max 3 secs)')
            try:
                self.listener.waitForTransform(frame, self.world_link, now, rospy.Duration(3))
                chain = self.listener.chain(frame, now, self.world_link, now, self.world_link)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr('Could not get transform from ' + frame + ' to ' + self.world_link + '(max 3 secs)')
                continue

            for idx in range(0, len(chain) - 1):
                parent = chain[idx]
                child = chain[idx + 1]
                transforms_list.append({'parent': parent, 'child': child, 'key': generateKey(parent, child)})

        # https://stackoverflow.com/questions/31792680/how-to-make-values-in-list-of-dictionary-unique
        uniq_l = list(map(dict, frozenset(frozenset(i.items()) for i in transforms_list)))
        return uniq_l  # get unique values
