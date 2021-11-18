#!/usr/bin/env python
import glob
import sys
import os.path
import argparse
import numpy as np
import json
import pandas
from colorama import Fore, Style
from shutil import copyfile
import cv2
from tqdm import tqdm
import re
from atom_calibration.collect import patterns

from atom_core.naming import generateKey
from tf.transformations import quaternion_from_matrix, quaternion_from_euler


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
    ap.add_argument("-ds", "--dataset", help="Path containing the dataset.", type=str, required=True)
    ap.add_argument("-json", "--json_file", help="Json file containing config file.", type=str, required=True)
    ap.add_argument("-out", "--dataset_out", type=str, required=True, help="Full path to the output dataset folder")
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
    # --- Calibration_config dictionary
    # ---------------------------------------
    D = {}  # initialize the dataset
    with open(args['json_file'], 'r') as f:
        config = json.load(f)
        D['calibration_config'] = config
    print("\n")

    # ---------------------------------------
    # --- Sensors dictionary
    # ---------------------------------------
    sensors = {}
    for sensor_name in config['sensors']:

        extensions = ("*.png", "*.jpg", "*.jpeg", "*.tiff")
        image_paths = []
        for extension in extensions:
            image_paths.extend(glob.glob(args['dataset'] + '/images/' + sensor_name + '/' + extension))

        image_paths.sort()  # sort the list of image paths

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

        dy = config['calibration_pattern']['dimension']['y']
        dx = config['calibration_pattern']['dimension']['x']
        objp = np.zeros((dy * dx, 3), np.float32)
        objp[:, :2] = np.mgrid[0:dx, 0:dy].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        print('Detecting chessboard corners for images from sensor ' + sensor_name + '. It may take a while ...')
        height = 0
        width = 0
        for image_path in tqdm(image_paths):
            img = cv2.imread(image_path)
            window_name = os.path.basename(image_path)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (dx, dy), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (dx, dy), corners2, ret)

                # cv2.namedWindow(window_name, cv2.WINDOW_GUI_EXPANDED)
                # cv2.imshow(window_name, img)
                # cv2.resizeWindow(window_name, width / 3, height / 3)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

        print('Running intrinsic calibration with ' + str(len(image_paths)) + ' images ... It may take a while ... ')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        tmp = mtx.reshape((1, 9)).tolist()
        K = tmp[0]
        tmp = dist.tolist()
        Dist = tmp[0]

        # extract P from K
        P = K[0:3]
        P.append(0)
        P.extend(K[3:6])
        P.append(0)
        P.extend(K[6:9])
        P.append(0)

        R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

        camera_info = {'K': K, 'D': Dist, 'P': P, 'R': R, 'binning_x': 0, 'binning_y': 0,
                       'distortion_model': 'plump_bob',
                       'header': {'frame_id': sensor_name + '_optical_frame', 'stamp': {'secs': 1, 'nsecs': 0},
                                  'seq': 0},
                       'height': height, 'width': width,
                       'roi': {'do_rectify': False, 'height': 0, 'width': 0, 'x_offset': 0, 'y_offset': 0}}

        chain = [{'child': 'ee_link', 'parent': 'base_link',
                  'key': generateKey('base_link', 'ee_link')},
                 {'child': sensor_name, 'parent': 'ee_link',
                  'key': generateKey('ee_link', sensor_name)},
                 {'child': sensor_name + '_optical_frame', 'parent': sensor_name,
                  'key': generateKey(sensor_name, sensor_name + '_optical_frame')}
                 ]

        sensor = {'_name': sensor_name,
                  'calibration_parent': config['sensors'][sensor_name]['parent_link'],
                  'calibration_child': config['sensors'][sensor_name]['child_link'],
                  'parent': sensor_name + '_optical_frame',
                  'camera_info': camera_info,
                  'camera_info_topic': '/' + sensor_name + '/camera_info',
                  'chain': chain,
                  'msg_type': 'Image',
                  'topic_name': '/' + sensor_name + '/image_color'}

        sensors[sensor_name] = sensor

    D['sensors'] = sensors

    # ---------------------------------------
    # --- # Copy images from input to output dataset.
    # ---------------------------------------

    print('Copying ' + str(len(image_paths)) + ' images...')
    for sensor_name in config['sensors']:

        extensions = ("*.png", "*.jpg", "*.jpeg", "*.tiff")
        image_paths = []
        for extension in extensions:
            image_paths.extend(glob.glob(args['dataset'] + '/images/' + sensor_name + '/' + extension))
        image_paths.sort()  # sort the list of image paths

        for image_path in image_paths:
            filename, file_extension = os.path.splitext(os.path.basename(image_path))
            numerics = re.findall(r'\d+', filename)
            image_idx = str(int(numerics[-1]))  # get the last of the numbers in the filename and use it as index
            filename_out = args['dataset_out'] + '/' + sensor_name + '_' + str(image_idx) + file_extension
            print('Copy image ' + image_path + ' to ' + filename_out)
            copyfile(image_path, filename_out)

    # ---------------------------------------
    # --- Collections dictionary
    # ---------------------------------------

    # Load robot_cali.txt file
    robot_cali_filename = args['dataset'] + '/robot_cali.txt'
    robot_poses_vec = pandas.read_csv(robot_cali_filename, sep="\t", header=None)
    robot_poses_vec = robot_poses_vec.to_numpy()  # convert from pandas dataframe to np array

    # Create a pattern detector for usage later
    pattern = D['calibration_config']['calibration_pattern']
    if pattern['pattern_type'] == 'chessboard':
        pattern_detector = patterns.ChessboardPattern(pattern['dimension'], pattern['size'])
    elif pattern['pattern_type'] == 'charuco':
        pattern_detector = patterns.CharucoPattern(pattern['dimension'], pattern['size'], pattern['inner_size'])
    else:
        raise ValueError('Unknown pattern type ' + pattern['pattern_type'])

    # Setup dictionaries with empty collections
    collections = {}
    for sensor_name in config['sensors']:
        extensions = ("*.png", "*.jpg", "*.jpeg", "*.tiff")
        image_paths = []
        for extension in extensions:
            image_paths.extend(glob.glob(args['dataset'] + '/images/' + sensor_name + '/' + extension))
        image_paths.sort()  # sort the list of image paths

        for image_path in image_paths:
            filename, file_extension = os.path.splitext(os.path.basename(image_path))
            numerics = re.findall(r'\d+', filename)
            image_idx = numerics[-1]  # get the last of the numbers in the filename and use it as index
            key = str(int(image_idx))

            if not key in collections:
                collections[key] = {'transforms':{}, 'data': {}, 'labels':{}}

    # Create transforms dictionary
    for collection_key in collections:

        transforms = {}  # initialize the transforms dictionary

        # Transform: base_link-ee_link
        # Must transform from their format in the txt file into ours
        row_number = 1 + int(collection_key) * 4  # they started counting at 1 ... who are these people ???
        T = []
        for row in range(row_number, row_number + 4):
            tmp = robot_poses_vec[row].tolist()
            tmp = [float(x) for x in tmp[0].split()]
            T.extend(tmp)
        T = np.linalg.inv(np.asarray(T).reshape((4, 4)))

        quat = list(quaternion_from_matrix(T))
        trans = list(T[0:3, 3]/1000.0)
        parent = config['world_link']
        child = 'ee_link'  # TODO Confirm ee_link with Eurico
        transform_key = generateKey(parent, child)
        transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
        transforms[transform_key] = transform

        for sensor_name in config['sensors']:
            # Transform: ee_link-hand_camera
            # TODO Talk to Eurico about this
            rpy = [-np.pi*0.5, -np.pi*0.5, np.pi]
            quat = list(quaternion_from_euler(rpy[0], rpy[1], rpy[2]))
            trans = [0, 0, 0]
            parent = 'ee_link'
            child = sensor_name
            transform_key = generateKey(parent, child)
            transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
            transforms[transform_key] = transform

            # Transform: camera-camera_optical_frame
            quat = [-.5, 0.5, -0.5, 0.5]  # optical transformation
            trans = [0, 0, 0]
            parent = sensor_name
            child = sensor_name + '_optical_frame'
            transform_key = generateKey(parent, child)
            transform = {'trans': trans, 'quat': quat, 'parent': parent, 'child': child}
            transforms[transform_key] = transform

            collections[collection_key]['transforms'] = transforms

    # Add data and labels for each collection and sensor
    for sensor_name in config['sensors']:
        print('Creating collections for sensor ' + sensor_name)

        extensions = ("*.png", "*.jpg", "*.jpeg", "*.tiff")
        image_paths = []
        for extension in extensions:
            image_paths.extend(glob.glob(args['dataset'] + '/images/' + sensor_name + '/' + extension))
        image_paths.sort()  # sort the list of image paths

        for image_path in image_paths:
            filename, file_extension = os.path.splitext(os.path.basename(image_path))
            numerics = re.findall(r'\d+', filename)
            image_idx = str(int(numerics[-1]))  # get the last of the numbers in the filename and use it as index

            filename_relative = sensor_name + '_' + image_idx + file_extension
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            h, w, _ = image.shape

            stamp = {'nsecs': 0, 'secs': image_idx}  # assume one collection per second
            data = {'data_file': filename_relative, 'encoding': 'rgb8',
                    'header': {'frame_id': sensor_name + '_optical_frame', 'stamp': stamp},
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

            collections[image_idx]['data'][sensor_name] = data
            collections[image_idx]['labels'][sensor_name] = labels
            print('Created collection ' + image_idx + ' of ' + str(len(image_paths) - 1))

    D['collections'] = collections
    createJSONFile(args['dataset_out'] + '/dataset.json', D)

