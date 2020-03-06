#!/usr/bin/env python
import glob
import sys
import os.path
import argparse

import json
import pandas
from colorama import Fore, Style
from shutil import copyfile
import cv2
from interactive_calibration import patterns

from OptimizationUtils.utilities import generateKey
from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_from_euler


def createJSONFile(output_file, D):
    print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
    f = open(output_file, 'w')
    json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
    print >> f, json.dumps(D, indent=2, sort_keys=True)
    f.close()
    print("Completed.")


if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-rwhe", "--rwhe_dataset", help="Path containing the RWHE dataset.", type=str, required=True)
    ap.add_argument("-json", "--json_file", help="Json file containing config file.", type=str, required=True)
    ap.add_argument("-out", "--dataset_out", type=str, required=True, help="Full path to the output dataset folder")
    ap.add_argument("-s", "--sensor", help="This problem uses a single sensor. This argument defines its name.",
                    type=str, required=True)
    args = vars(ap.parse_args())

    # print('Copying images ...')
    # Testing output folder
    if not os.path.exists(args['dataset_out']):
        os.mkdir(args['dataset_out'])  # Create the new folder
    else:
        while True:
            msg = Fore.YELLOW + "To continue, the directory '{}' will be deleted.\n"
            msg = msg + "Do you wish to continue? [y/N] " + Style.RESET_ALL

            answer = raw_input(msg.format(args['dataset_out']))
            if len(answer) > 0 and answer[0].lower() in ('y', 'n'):
                if answer[0].lower() == 'n':
                    sys.exit(1)
                else:
                    break
            else:
                sys.exit(1)  # defaults to N

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------

    # Copy images from input to output dataset.
    image_paths = glob.glob(args['rwhe_dataset'] + '/*.png')
    image_paths.sort()
    print('Copying ' + str(len(image_paths)) + ' images...')
    for image_path in image_paths:
        image_idx = os.path.basename(image_path)[:-4]
        filename_out = args['dataset_out'] + '/' + args['sensor'] + '_' + image_idx + '.png'
        print('Copy image ' + image_path + ' to ' + filename_out)
        copyfile(image_path, filename_out)

    # Load RobotPosesVec.txt
    robot_poses_vec_filename = args['rwhe_dataset'] + '/RobotPosesVec.txt'
    robot_poses_vec = pandas.read_csv(robot_poses_vec_filename, sep="\t", header=None)
    robot_poses_vec = robot_poses_vec.to_numpy()  # convert from pandas dataframe to np array

    D = {}  # initialize the dataset

    # ---------------------------------------
    # --- Calibration_config dictionary
    # ---------------------------------------
    with open(args['json_file'], 'r') as f:
        config = json.load(f)
        D['calibration_config'] = config
    print("\n")

    # Create a pattern detector for usage later
    pattern = D['calibration_config']['calibration_pattern']
    if pattern['pattern_type'] == 'chessboard':
        pattern_detector = patterns.ChessboardPattern(pattern['dimension'], pattern['size'])
    elif pattern['pattern_type'] == 'charuco':
        pattern_detector = patterns.CharucoPattern(pattern['dimension'], pattern['size'], pattern['inner_size'])
    else:
        raise ValueError('Unknown pattern type ' + pattern['pattern_type'])

    # ---------------------------------------
    # --- Collections dictionary
    # ---------------------------------------
    collections = {}
    for idx, image_path in enumerate(image_paths):
        image_idx = os.path.basename(image_path)[:-4]
        filename_out = args['dataset_out'] + '/' + args['sensor'] + '_' + image_idx + '.png'
        filename_relative = args['sensor'] + '_' + image_idx + '.png'

        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        h, w, _ = image.shape

        stamp = {'nsecs': 0, 'secs': idx}  # assume one collection per second
        data = {'data_file': filename_out, 'encoding': 'rgb8',
                'header': {'frame_id': args['sensor'] + '_optical_frame', 'stamp': stamp},
                'height': h, 'width': w, 'step': w, 'is_bigendian': 0
                }

        # Detect pattern
        result = pattern_detector.detect(image)
        if result['detected']:
            c = []
            if result.has_key('ids'):
                # The charuco pattern also return an ID for each keypoint.
                # We can use this information for partial detections.
                for idx, corner in enumerate(result['keypoints']):
                    c.append({'x': float(corner[0][0]), 'y': float(corner[0][1]), 'id': result['ids'][idx]})
            else:
                for corner in result['keypoints']:
                    c.append({'x': float(corner[0][0]), 'y': float(corner[0][1])})

            # Update the dictionary with the labels
            labels = {'detected': True, 'idxs': c}
        else:
            labels = {'detected': False, 'idxs': []}

        # Create transforms dictionary
        transforms = {}  # initialize the transforms dictionary

        # Transform: base_link-ee_link
        row_number = int(image_idx) - 1  # they started counting at 1 ... who are these people ???
        T = robot_poses_vec[row_number, :].reshape(4, 4)
        quat = list(quaternion_from_matrix(T))
        trans = list(T[0:3, 3])
        parent = config['world_link']
        child = 'ee_link'  # TODO Confirm ee_link with Eurico
        transform_key = generateKey(parent, child)
        transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
        transforms[transform_key] = transform

        # Transform: ee_link-hand_camera
        rpy = pattern['origin'][3:]
        quat = list(quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='sxyz'))
        trans = pattern['origin'][0:3]
        parent = 'ee_link'
        child = args['sensor']
        transform_key = generateKey(parent, child)
        transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
        transforms[transform_key] = transform

        # Transform: camera-camera_optical_frame
        quat = [-.5, 0.5, -0.5, 0.5]  # optical transformation
        trans = [0, 0, 0]
        parent = args['sensor']
        child = args['sensor'] + '_optical_frame'
        transform_key = generateKey(parent, child)
        transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
        transforms[transform_key] = transform

        collection = {'data': {}, 'labels': {}, 'transforms': {}}
        collection['data'][args['sensor']] = data
        collection['labels'][args['sensor']] = labels
        collection['transforms'] = transforms

        collection_idx = str(image_idx)
        collections[collection_idx] = collection
        print('Created collection ' + collection_idx + ' of ' + str(len(image_paths)))

    D['collections'] = collections

    # ---------------------------------------
    # --- Sensors dictionary
    # ---------------------------------------
    sensors = {}
    name = args['sensor']

    # Values from Matlab
    # cameraIntrinsics with properties:
    #   FocalLength: [2.0587e+03 2.0593e+03]
    #   PrincipalPoint: [962.1690 611.0970]
    #   ImageSize: [1208 1928]
    #   RadialDistortion: [-0.1163 0.1688 - 0.0722]
    #   TangentialDistortion: [2.8333e-04 2.1871e-04]
    #   Skew: 0.1682
    K = [2058.7, 0, 962.1690, 0, 2059.3, 611.0970, 0, 0, 1]
    Dist = [-0.1163, 0.1688, 2.8333e-04, 2.1871e-04, -0.0722]  # We use distortion_coefficients = (k1,k2,p1,p2,k3) as
    # in opencv
    P = [2058.7, 0, 962.1690, 0, 0, 2059.3, 611.0970, 0, 0, 0, 1, 0]
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    camera_info = {'K': K, 'D': Dist, 'P': P, 'R': R, 'binning_x': 0, 'binning_y': 0, 'distortion_model': 'plump_bob',
                   'header': {'frame_id': name + '_optical_frame', 'stamp': {'secs': 1, 'nsecs': 0}, 'seq': 0},
                   'height': 1208, 'width': 1928,
                   'roi': {'do_rectify': False, 'height': 0, 'width': 0, 'x_offset': 0, 'y_offset': 0}}

    chain = [{'child': 'ee_link', 'parent': 'base_link',
              'key': generateKey('base_link', 'ee_link')},
             {'child': name, 'parent': 'ee_link',
              'key': generateKey('ee_link', name)},
             {'child': name + '_optical_frame', 'parent': name,
              'key': generateKey(name, name + '_optical_frame')}
             ]

    sensor = {'_name': name,
              'calibration_parent': config['sensors'][name]['parent_link'],
              'calibration_child': config['sensors'][name]['child_link'],
              'parent': name + '_optical_frame',
              'camera_info': camera_info,
              'camera_info_topic': '/' + name + '/camera_info',
              'chain': chain,
              'msg_type': 'Image',
              'topic_name': '/' + name + '/image_color'}

    sensors[name] = sensor

    # Create top level dictionary and save to file
    D['sensors'] = sensors
    createJSONFile(args['dataset_out'] + '/data_collected.json', D)
