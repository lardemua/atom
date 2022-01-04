import copy
# stdlib
import functools
import json
# 3rd-party
import numpy as np
import os

# 3rd-party
# import pypcd
from colorama import Fore, Style

import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import random
import atom_core.pypcd as pypcd
from atom_core.config_io import uriReader
from atom_core.naming import generateName, generateKey
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
from rospy_message_converter import message_converter
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from atom_calibration.collect.label_messages import convertDepthImage32FC1to16UC1, convertDepthImage16UC1to32FC1
import imageio

def printImageInfo(image, text=None):
    if not text is None:
        print(text +
              '\n\tshape = ' + str(image.shape) +
              '\n\tdtype = ' + str(image.dtype) +
              '\n\tmax value = ' + str(np.nanmax(image)) +
              '\n\tmin value = ' + str(np.nanmin(image)))

def loadResultsJSON(json_file, collection_selection_function):
    # NOTE(eurico): I removed the URI reader because the argument is provided by the command line
    #   and our guide lines is to use environment variables, which the shell already expands.
    #   Furthermore, the URI resolver required an import from a `top-level` package which does
    #   not make sense in a `core` package.
    # Note(Miguel): but I think the expansion does not work, e.g. you can't do --dataset_folded ~/datasets/...
    # Also, the utilities are now in core. No dependency on another package anymore. We can discuss this more...
    json_file, _, _ = uriReader(json_file)

    f = open(json_file, 'r')
    dataset = json.load(f)
    dataset_folder = os.path.dirname(json_file)
    bridge = CvBridge()

    # Load images from files into memory. Images in the json file are stored in separate png files and in their place
    # a field "data_file" is saved with the path to the file. We must load the images from the disk.
    # Do the same for point clouds saved in pcd files
    skipped_loading = []
    for collection_key, collection in dataset['collections'].items():

        # Check if collection is listed to be ignored by csf and do not load image and point cloud if it is
        if not collection_selection_function is None:
            if not collection_selection_function(collection_key):  # use the lambda expression csf
                skipped_loading.append(collection_key)
                continue

        for sensor_key, sensor in dataset['sensors'].items():

            if not (sensor['modality'] == 'rgb' or sensor['modality'] == 'lidar3d' or sensor['modality'] == 'lidar2d' or
                    sensor[
                        'modality'] == 'depth'):
                continue  # only process images or point clouds

            # Check if we really need to load the file.
            if 'data' in collection['data'][sensor_key]:
                load_file = False
            elif 'data_file' in collection['data'][sensor_key]:
                filename = dataset_folder + '/' + collection['data'][sensor_key]['data_file']
                if os.path.isfile(filename):
                    load_file = True
                else:
                    raise ValueError('Datafile points to ' + collection['data'][sensor_key]['data_file'] +
                                     ' but file ' + filename + ' does not exist.')
            else:
                raise ValueError('Dataset does not contain data nor data_file folders.')

            if load_file and (sensor['modality'] == 'rgb'):  # Load image.
                filename = os.path.dirname(json_file) + '/' + collection['data'][sensor_key]['data_file']
                cv_image = cv2.imread(filename)
                collection['data'][sensor_key].update(getDictionaryFromCvImage(cv_image))

            elif load_file and sensor['modality'] == 'depth':
                filename = os.path.dirname(json_file) + '/' + collection['data'][sensor_key]['data_file']

                print(collection['data'][sensor_key]['header']['frame_id'])
                cv_image_int16_tenths_of_millimeters = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
                cv_image_float32_meters = convertDepthImage16UC1to32FC1(cv_image_int16_tenths_of_millimeters, scale=10000.0)

                printImageInfo(cv_image_int16_tenths_of_millimeters, text='cv_image_int16_tenths_of_millimeters')
                printImageInfo(cv_image_float32_meters, text='cv_image_float32_meters')

                # collection['data'][sensor_key].update(getDictionaryFromDepthImage(cv_image_float32_meters))

                msg = getDictionaryFromDepthImage(cv_image_float32_meters)
                collection['data'][sensor_key]['data'] = msg['data']
                # TODO eliminate data_file
                # TODO Why this is not needed for rgb? Should be done as well

                print(collection['data'][sensor_key]['header']['frame_id'])
                # print(collection['data'][sensor_key].keys())
                # TODO verify if values in the dataset or ok
                # exit(0)

            elif load_file and (
                    sensor['modality'] == 'lidar3d' or sensor['modality'] == 'lidar2d'):  # Load point cloud.
                filename = os.path.dirname(json_file) + '/' + collection['data'][sensor_key]['data_file']
                frame_id = str(collection['data'][sensor_key]['header']['frame_id'])

                # setup header for point cloud from existing dictionary data
                header = Header()
                header.frame_id = frame_id
                time = rospy.Time()
                time.secs = collection['data'][sensor_key]['header']['stamp']['secs']
                time.nsecs = collection['data'][sensor_key]['header']['stamp']['nsecs']
                header.stamp = time
                header.seq = collection['data'][sensor_key]['header']['seq']

                # read point cloud from dist
                msg = read_pcd(filename, cloud_header=header)

                # convert to dictionary
                collection['data'][sensor_key].update(message_converter.convert_ros_message_to_dictionary(msg))

    print('Skipped loading images and point clouds for collections: ' + str(skipped_loading) + '.')
    return dataset, json_file


def saveResultsJSON(output_file, dataset_in, freeze_dataset=False):
    if freeze_dataset:  # to make sure our changes only affect the dictionary to save
        dataset = copy.deepcopy(dataset_in)
    else:
        dataset = dataset_in

    output_folder = os.path.dirname(output_file)
    bridge = CvBridge()

    # Process the dataset to remove data from the data fields and, if needed, write the files.
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            createDataFile(dataset_in, collection_key, sensor, sensor_key, output_folder)

        # Do the same for additional data topics ...
        # for description, sensor in dataset['additional_sensor_data'].items():
        #     createDataFile(dataset_in, collection_key, sensor, description, output_folder, 'additional_data')

    createJSONFile(output_file, dataset)  # write dictionary to json


def createDataFile(dataset, collection_key, sensor, sensor_key, output_folder, data_type='data'):
    if not (sensor['modality'] == 'rgb' or sensor['modality'] == 'lidar3d' or sensor['modality'] == 'lidar2d' or sensor[
        'modality'] == 'depth'):
        return

    # Check if data_file has to be created based on the existence of the field 'data_file' and the file itself.
    if 'data_file' in dataset['collections'][collection_key][data_type][sensor_key]:
        filename = output_folder + '/' + dataset['collections'][collection_key][data_type][sensor_key]['data_file']
        if os.path.isfile(filename):
            create_data_file = False
            if 'data' in dataset['collections'][collection_key][data_type][sensor_key]:  # remove the data field
                del dataset['collections'][collection_key][data_type][sensor_key]['data']
        else:
            create_data_file = True
    else:
        create_data_file = True

    if create_data_file:
        print(
            'Collection ' + str(collection_key) + '. Creating data file for sensor ' + str(sensor_key) + ' msg type ' +
            sensor[
                'msg_type'])
    # TODO add if for depth modality
    if create_data_file and sensor['modality'] == 'rgb':  # save image.
        # Save image to disk if it does not exist
        filename = output_folder + '/' + sensor['_name'] + '_' + str(collection_key) + '.jpg'
        if not os.path.isfile(filename):  # Write pointcloud to pcd file
            cv_image = getCvImageFromDictionary(dataset['collections'][collection_key][data_type][sensor_key])
            cv2.imwrite(filename, cv_image)
            print('Saved file ' + filename + '.')

        # Add data_file field, and remove data field
        filename_relative = sensor['_name'] + '_' + str(collection_key) + '.jpg'
        dataset['collections'][collection_key][data_type][sensor_key][
            'data_file'] = filename_relative  # add data_file field
        if 'data' in dataset['collections'][collection_key][data_type][sensor_key]:  # Delete data field from dictionary
            del dataset['collections'][collection_key][data_type][sensor_key]['data']

    elif create_data_file and sensor['modality'] == 'lidar3d' or sensor['modality'] == 'lidar2d':  # save point cloud
        # Save file if it does not exist
        filename = output_folder + '/' + sensor['_name'] + '_' + str(collection_key) + '.pcd'
        if not os.path.isfile(filename):  # Write pointcloud to pcd file
            # from: dictionary -> ros_message -> PointCloud2() -> pcd file
            msg = getPointCloudMessageFromDictionary(dataset['collections'][collection_key][data_type][sensor_key])
            write_pcd(filename, msg)
            print('Saved file ' + filename + '.')

        # Add data_file field, and remove data field
        filename_relative = sensor['_name'] + '_' + str(collection_key) + '.pcd'
        dataset['collections'][collection_key][data_type][sensor_key][
            'data_file'] = filename_relative  # add data_file field
        if 'data' in dataset['collections'][collection_key][data_type][sensor_key]:  # Delete data field from dictionary
            del dataset['collections'][collection_key][data_type][sensor_key]['data']

    elif create_data_file and sensor['modality'] == 'depth':
        # print('i dont know what im doing')
        # Save image to disk if it does not exist
        filename = output_folder + '/' + sensor['_name'] + '_' + str(collection_key) + '.png'
        # filename_np=output_folder + '/' + sensor['_name'] + '_' + str(collection_key) + '.npy'
        if not os.path.isfile(filename):  # Write pointcloud to pcd file
            cv_image = getCvImageFromDictionaryDepth(dataset['collections'][collection_key][data_type][sensor_key])
            print("image from getimage")
            print(cv_image.dtype)
            cv_image = convertDepthImage32FC1to16UC1(cv_image, scale=10000) # Better to use tenths of milimeters
            # cv2.normalize(cv_image, cv_image, 0, 65535, cv2.NORM_MINMAX)
            # cv2.imwrite(filename, cv_image)
            imageio.imwrite(filename, cv_image)
            print('Saved file ' + filename + '.')

        # Add data_file field, and remove data field
        filename_relative = sensor['_name'] + '_' + str(collection_key) + '.png'
        dataset['collections'][collection_key][data_type][sensor_key][
            'data_file'] = filename_relative  # add data_file field
        if 'data' in dataset['collections'][collection_key][data_type][sensor_key]:  # Delete data field from dictionary
            del dataset['collections'][collection_key][data_type][sensor_key]['data']


def getDictionaryFromCvImage(cv_image):
    """
    Creates a dictionary from the opencv image, going from cvimage -> ros_message -> dictionary.
    :param cv_image:  the image in opencv format.
    :return: A dictionary converted from the ros message.
    """
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    return message_converter.convert_ros_message_to_dictionary(msg)

def getDictionaryFromDepthImage(cv_image):
    """
    Creates a dictionary from the opencv image, going from cvimage -> ros_message -> dictionary.
    :param cv_image:  the image in opencv format.
    :return: A dictionary converted from the ros message.
    """
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
    return message_converter.convert_ros_message_to_dictionary(msg)


def getCvImageFromDictionary(dictionary_in, safe=False):
    """
    Converts a dictionary (read from a json file) into an opencv image.
    To do so it goes from dictionary -> ros_message -> cv_image
    :param dictionary_in: the dictionary read from the json file.
    :return: an opencv image.
    """
    if safe:
        d = copy.deepcopy(dictionary_in)  # to make sure we don't touch the dictionary
    else:
        d = dictionary_in

    if 'data_file' in d:  # Delete data field from dictionary
        del d['data_file']  # will disrupt the dictionary to ros message

    msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/Image', d)
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(msg)


def getCvImageFromDictionaryDepth(dictionary_in, safe=False):
    """
    Converts a dictionary (read from a json file) into an opencv image.
    To do so it goes from dictionary -> ros_message -> cv_image
    :param dictionary_in: the dictionary read from the json file.
    :return: an opencv image.
    """
    if safe:
        d = copy.deepcopy(dictionary_in)  # to make sure we don't touch the dictionary
    else:
        d = dictionary_in

    if 'data_file' in d:  # Delete data field from dictionary
        del d['data_file']  # will disrupt the dictionary to ros message

    msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/Image', d)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # print("image inside get image: ")
    # print(image.dtype)
    return image


def getPointCloudMessageFromDictionary(dictionary_in):
    """
    Converts dictionary to PointCloud2 message.
    :param dictionary_in: dictionary.
    :return: a ros Pointcloud2 message.
    """
    d = copy.deepcopy(dictionary_in)  # to make sure we don't touch the dictionary

    if 'data_file' in d:  # Delete data field from dictionary
        del d['data_file']  # will disrupt the dictionary to ros message

    msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/PointCloud2', d)
    return msg


class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NpEncoder, self).default(obj)


# json.dumps(data, cls=NpEncoder)

def createJSONFile(output_file, input):
    """
    Creates the json file containing the results data.
    :param output_file: output file.
    :param input: input dictionary containing the data.
    """
    D = copy.deepcopy(input)
    walk(D)

    print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
    f = open(output_file, 'w')
    json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
    # print >> f, json.dumps(D, indent=2, sort_keys=True)
    f.write(json.dumps(D, indent=2, sort_keys=True, cls=NpEncoder))

    f.close()
    print("Completed.")


def is_jsonable(x):
    try:
        json.dumps(x)
        return True
    except (TypeError, OverflowError):
        return False


def walk(node):
    for key, item in node.items():
        if isinstance(item, dict):
            walk(item)
        else:
            if isinstance(item, np.ndarray) and key == 'data':  # to avoid saving images in the json
                del node[key]

            elif isinstance(item, np.ndarray):
                node[key] = item.tolist()
            pass


def write_pcd(filename, pointcloud, mode='binary'):
    """
    This is meant to replace the old read_pcd from Andre which broke when migrating to python3.
    :param filename:
    :param cloud_header:
    :return:
    """

    print('Reading point cloud from ' + Fore.BLUE + filename + Style.RESET_ALL)
    pc = pypcd.PointCloud.from_msg(pointcloud)
    pc.save_pcd(filename, compression=mode)


def read_pcd(filename, cloud_header=None, verbose=False):
    """
    This is meant to replace the old read_pcd from Andre which broke when migrating to python3.
    :param filename:
    :param cloud_header:
    :return:
    """
    if not os.path.isfile(filename):
        raise Exception("[read_pcd] File does not exist.")

    if verbose:
        print('Reading point cloud from ' + Fore.BLUE + filename + Style.RESET_ALL)
    pc = pypcd.PointCloud.from_path(filename)

    cloud = pc.to_msg()
    if cloud_header is not None:
        # cloud.header = header
        cloud.header = cloud_header
        # print('This is it, header is ' + str(cloud_header))
    else:
        cloud.header.frame_id = "/pcd_cloud"

    return cloud


def genCollectionPrefix(collection_key, string):
    """ Standardized form of deriving a name with a collection related prefix. """
    return generateName(string, prefix='c' + str(collection_key), separator='_')
    # return 'c' + str(collection_key) + '_' + str(string)


def filterSensorsFromDataset(dataset, args):
    """
    Filters some sensors from the dataset, using a couple of arguments in arg
    :param dataset:
    :param args: Makes use of 'sensor_selection_function'
    """

    if not args['sensor_selection_function'] is None:
        deleted = []
        for sensor_key in dataset['sensors']:
            if not args['sensor_selection_function'](sensor_key):  # use the lambda expression ssf
                deleted.append(sensor_key)

        for sensor_key in deleted:
            del dataset['sensors'][sensor_key]

        print("Deleted sensors: " + str(deleted))

    if not dataset['sensors'].keys():
        raise ValueError('No sensors were selected. Cannot optimize without sensors. Please revise your '
                         'dataset and your sensor selection function.')

    return dataset


def filterCollectionsFromDataset(dataset, args):
    """
    Filters some collections from the dataset, using a couple of arguments in arg
    :param dataset:
    :param args: Makes use of 'collection_selection_function', 'use_incomplete_collections' and
                'remove_partial_detections'
    """

    if not args['collection_selection_function'] is None:
        deleted = []
        for collection_key in dataset['collections'].keys():
            if not args['collection_selection_function'](collection_key):  # use the lambda expression csf
                deleted.append(collection_key)

        for collection_key in deleted:
            del dataset['collections'][collection_key]
        print('Deleted collections: ' + str(deleted) + ' because of the -csf flag.')

    if not args['use_incomplete_collections']:
        deleted = []
        # Deleting collections where the pattern is not found by all sensors:
        for collection_key, collection in dataset['collections'].items():
            for sensor_key, sensor in dataset['sensors'].items():
                if not collection['labels'][sensor_key]['detected']:
                    deleted.append(collection_key)
                    break

        for collection_key in deleted:
            del dataset['collections'][collection_key]
        print('Deleted collections: ' + str(deleted) + ' because these are incomplete. If you want to use them set the '
                                                       'use_incomplete_collections flag.')

    if args['remove_partial_detections']:
        number_of_corners = int(dataset['calibration_config']['calibration_pattern']['dimension']['x']) * \
                            int(dataset['calibration_config']['calibration_pattern']['dimension']['y'])
        # Deleting labels in which not all corners are found:
        for collection_key, collection in dataset['collections'].items():
            for sensor_key, sensor in dataset['sensors'].items():
                if sensor['msg_type'] == 'Image' and collection['labels'][sensor_key]['detected']:
                    if not len(collection['labels'][sensor_key]['idxs']) == number_of_corners:
                        print(
                            Fore.RED + 'Partial detection removed:' + Style.RESET_ALL + ' label from collection ' +
                            collection_key + ', sensor ' + sensor_key)
                        collection['labels'][sensor_key]['detected'] = False

    # It may occur that some collections do not have any detection in a camera sensor (because all detections were
    # partial and have been removed, or just because no detection existed). Since we need at lease one camera sensor
    # detection of the pattern in a collection in order initialize the parameters (check calibrate line 133),
    # we will remove collections which do not have at least one detection by a camera.
    flag_have_cameras = False  # do this only if we have at least one camera in the sensor list.
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['msg_type'] == 'Image':
            flag_have_cameras = True
            break

    if flag_have_cameras:
        deleted = []
        for collection_key, collection in dataset['collections'].items():
            flag_have_at_least_one_camera_detection = False
            for sensor_key, sensor in dataset['sensors'].items():
                if sensor['msg_type'] == 'Image' and collection['labels'][sensor_key]['detected']:
                    flag_have_at_least_one_camera_detection = True

            if not flag_have_at_least_one_camera_detection:  # delete collection without detection by cameras.

                deleted.append(collection_key)

        for collection_key in deleted:
            del dataset['collections'][collection_key]
        print('Deleted collections: ' + str(deleted) + ': at least one detection by a camera should be present.')

    if not dataset['collections'].keys():
        raise ValueError('No collections were selected. Cannot optimize without collections. Please revise your '
                         'dataset and your collection selection function.')

    print('After filtering, will use ' + str(len(dataset['collections'].keys())) + ' collections: ' + str(
        list(dataset['collections'].keys())))

    return dataset


def addNoiseToInitialGuess(dataset, args, selected_collection_key):
    """
    Adds noise
    :param dataset:
    :param args: Makes use of nig, i.e., the amount of noise to add to the initial guess atomic transformations to be
                 calibrated
    """
    if args['sample_seed'] is not None:
        np.random.seed(args['sample_seed'])

    nig_trans = args['noisy_initial_guess'][0]
    nig_rot = args['noisy_initial_guess'][1]

    for sensor_key, sensor in dataset['sensors'].items():
        if sensor_key == dataset['calibration_config']['anchored_sensor']:
            continue

        calibration_child = sensor['calibration_child']
        calibration_parent = sensor['calibration_parent']
        tf_link = generateKey(calibration_parent, calibration_child, suffix='')

        # Get original transformation
        quat = dataset['collections'][selected_collection_key]['transforms'][tf_link]['quat']
        translation = dataset['collections'][selected_collection_key]['transforms'][tf_link]['trans']
        euler_angles = tf.transformations.euler_from_quaternion(quat)

        # Add noise to the 6 pose parameters
        v = np.random.uniform(-1.0, 1.0, 3)
        v = v / np.linalg.norm(v)
        new_translation = translation + v * nig_trans

        v = np.random.choice([-1.0, 1.0], 3) * nig_rot
        new_angles = euler_angles + v

        # Replace the original atomic transformations by the new noisy ones
        new_quat = tf.transformations.quaternion_from_euler(new_angles[0], new_angles[1], new_angles[2])
        dataset['collections'][selected_collection_key]['transforms'][tf_link]['quat'] = new_quat
        dataset['collections'][selected_collection_key]['transforms'][tf_link]['trans'] = list(new_translation)

        # Copy randomized transform to all collections
        for collection_key, collection in dataset['collections'].items():
            dataset['collections'][collection_key]['transforms'][tf_link]['quat'] = \
                dataset['collections'][selected_collection_key]['transforms'][tf_link]['quat']
            dataset['collections'][collection_key]['transforms'][tf_link]['trans'] = \
                dataset['collections'][selected_collection_key]['transforms'][tf_link]['trans']
