import copy
# stdlib
import json
# 3rd-party
import numpy as np
import os

# 3rd-party
from colorama import Fore, Style

import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2
from atom_core.config_io import uriReader
from atom_core.naming import generateName
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
from rospy_message_converter import message_converter
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def loadResultsJSON(json_file):
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
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():

            if not (sensor['msg_type'] == 'Image' or sensor['msg_type'] == 'PointCloud2'):
                continue  # only process images or point clouds

            # Check if we really need to load the file.
            if collection['data'][sensor_key].has_key('data'):
                load_file = False
            elif collection['data'][sensor_key].has_key('data_file'):
                filename = dataset_folder + '/' + collection['data'][sensor_key]['data_file']
                if os.path.isfile(filename):
                    load_file = True
                else:
                    raise ValueError('Datafile points to ' + collection['data'][sensor_key]['data_file'] +
                                     ' but file ' + filename + ' does not exist.')
            else:
                raise ValueError('Dataset does not contain data nor data_file folders.')

            if load_file and sensor['msg_type'] == 'Image':  # Load image.
                filename = os.path.dirname(json_file) + '/' + collection['data'][sensor_key]['data_file']
                cv_image = cv2.imread(filename)
                collection['data'][sensor_key].update(getDictionaryFromCvImage(cv_image))

            elif load_file and sensor['msg_type'] == 'PointCloud2':  # Load point cloud.
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
                msg = read_pcd(filename, cloud_header=header, get_tf=False)

                # convert to dictionary
                collection['data'][sensor_key].update(message_converter.convert_ros_message_to_dictionary(msg))

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
        for description, sensor in dataset['additional_sensor_data'].items():
            createDataFile(dataset_in, collection_key, sensor, description, output_folder, 'additional_data')

    createJSONFile(output_file, dataset)  # write dictionary to json


def createDataFile(dataset, collection_key, sensor, sensor_key, output_folder, data_type='data'):
    if not (sensor['msg_type'] == 'Image' or sensor['msg_type'] == 'PointCloud2'):
        return

    # Check if data_file has to be created based on the existence of the field 'data_file' and the file itself.
    if dataset['collections'][collection_key][data_type][sensor_key].has_key('data_file'):
        filename = output_folder + '/' + dataset['collections'][collection_key][data_type][sensor_key]['data_file']
        if os.path.isfile(filename):
            create_data_file = False
        else:
            create_data_file = True
    else:
        create_data_file = True

    if create_data_file and sensor['msg_type'] == 'Image':  # save image.
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
        if dataset['collections'][collection_key][data_type][sensor_key].has_key(
                'data'):  # Delete data field from dictionary
            del dataset['collections'][collection_key][data_type][sensor_key]['data']

    elif create_data_file and sensor['msg_type'] == 'PointCloud2':  # save point cloud
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
        if dataset['collections'][collection_key][data_type][sensor_key].has_key(
                'data'):  # Delete data field from dictionary
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

    if d.has_key('data_file'):  # Delete data field from dictionary
        del d['data_file']  # will disrupt the dictionary to ros message

    msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/Image', d)
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(msg, "bgr8")


def getPointCloudMessageFromDictionary(dictionary_in, safe=False):
    """
    Converts dictionary to PointCloud2 message.
    :param dictionary_in: dictionary.
    :return: a ros Pointcloud2 message.
    """
    if safe:
        d = copy.deepcopy(dictionary_in)  # to make sure we don't touch the dictionary
    else:
        d = dictionary_in

    if d.has_key('data_file'):  # Delete data field from dictionary
        del d['data_file']  # will disrupt the dictionary to ros message

    msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/PointCloud2', d)
    return msg


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
    print >> f, json.dumps(D, indent=2, sort_keys=True)
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
    string_array = lambda x: x.split()
    float_array = lambda x: [float(j) for j in x.split()]
    int_array = lambda x: [int(j) for j in x.split()]
    word = lambda x: x.strip()
    headers = [("VERSION", float),
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
            func = headers[zip(*headers)[0].index(f)][1]
            header[f] = func(v)
            headers.remove((f, func))
        data = pcdfile.read()
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
            pf = PointField()
            pf.count = count
            pf.offset = offset
            pf.name = field
            pf.datatype = size_type_to_datatype(size, type)
            cloud.fields.append(pf)
            offset += size * count

    cloud.data = data[0:cloud.row_step * cloud.height]
    if cloud_header is not None:
        # cloud.header = header
        cloud.header = cloud_header
        print('This is it, header is ' + str(cloud_header))
    else:
        cloud.header.frame_id = "/pcd_cloud"

    if get_tf:
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = header["VIEWPOINT"][0:3]
        tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z = header["VIEWPOINT"][3:]

        return cloud, tf

    return cloud


def getPCLData(dict, json_file):
    """ Loads pointcloud data either from a json file or from a pcd file."""
    if not dict.has_key('data'):
        # Read pointcloud from pcd file
        filename = os.path.dirname(json_file) + '/' + dict['data_file']
        frame_id = dict['header']['frame_id']
        cloud_msg = read_pcd(filename, frame_id, get_tf=False)
    else:
        # Read pointcloud from json file
        cloud_msg = message_converter.convert_dictionary_to_ros_message("sensor_msgs/PointCloud2", dict)

    return cloud_msg


def setPCLData(dict, sensor, collection_key, json_file):
    output_folder = os.path.dirname(json_file)
    filename = output_folder + '/' + sensor['_name'] + '_' + str(collection_key) + '.pcd'

    if dict.has_key('data_file') or os.path.exists(filename):
        pass
    else:
        filename_relative = sensor['_name'] + '_' + str(collection_key) + '.pcd'

        # Add pcd filename to dictionary
        dict['data_file'] = filename_relative

        # Write pointcloud to pcd file
        cloud_msg = dict['data']

        write_pcd(filename, cloud_msg)

    # Delete data field from dictionary
    if dict.has_key('data'):
        del dict['data']

    return dict


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
        for sensor_key in dataset['sensors'].keys():
            if not args['sensor_selection_function'](sensor_key):  # use the lambda expression ssf
                deleted.append(sensor_key)
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
                del dataset['collections'][collection_key]
        print('Deleted collections: ' + str(deleted) + ' because of the -csf flag.')

    if not args['use_incomplete_collections']:
        # Deleting collections where the pattern is not found by all sensors:
        for collection_key, collection in dataset['collections'].items():
            for sensor_key, sensor in dataset['sensors'].items():
                if not collection['labels'][sensor_key]['detected']:
                    print(
                            Fore.RED + "Removing collection " + collection_key + ' -> pattern was not found in sensor ' +
                            sensor_key + ' (incomplete collection).' + Style.RESET_ALL)
                    del dataset['collections'][collection_key]
                    break

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

    # It may occur that some collections do not have any detection in a camera sensor (because all detection were
    # partial and have been removed, or just because no detection existed). Since we need at lease one camera sensor
    # detection of the pattern in a collection in order initialize the parameters (check calibrate line 133),
    # we will remove collections which do not have at least one detection by a camera.
    flag_have_cameras = False  # do this only if we have at least one camera in the sensor list.
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['msg_type'] == 'Image':
            flag_have_cameras = True
            break

    if flag_have_cameras:
        for collection_key, collection in dataset['collections'].items():
            flag_have_at_least_one_camera_detection = False
            for sensor_key, sensor in dataset['sensors'].items():
                if sensor['msg_type'] == 'Image' and collection['labels'][sensor_key]['detected']:
                    flag_have_at_least_one_camera_detection = True

            if not flag_have_at_least_one_camera_detection:  # delete collection without detection by cameras.
                print(Fore.RED + "Removing collection " + collection_key + Style.RESET_ALL +
                      ': at least one detection by a camera should be present.')
                del dataset['collections'][collection_key]

    if not dataset['collections'].keys():
        raise ValueError('No collections were selected. Cannot optimize without collections. Please revise your '
                         'dataset and your collection selection function.')

    print('After filtering, will use ' + str(len(dataset['collections'].keys())) + ' collections: ' + str(
        dataset['collections'].keys()))

    return dataset
