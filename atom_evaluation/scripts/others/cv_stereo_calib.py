#!/usr/bin/env python

"""
Stereo calibration from opencv
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import numpy as np
import cv2
import argparse
import json
import os

from colorama import Style, Fore
from collections import OrderedDict


def cvStereoCalibrate(objp, images_right, images_left):
    images_left.sort()
    images_right.sort()

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints_l = []  # 2d points in image plane.
    imgpoints_r = []  # 2d points in image plane.
    allids_l = []  # detect corner ids for left camera
    allids_r = []  # detect corner ids for right camera

    for i, fname in enumerate(images_right):
        # Read stereo images
        img_l = cv2.imread(images_left[i])
        img_r = cv2.imread(images_right[i])

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        n_points = int(dataset['calibration_config']['calibration_pattern']['dimension']['x']) * \
                   int(dataset['calibration_config']['calibration_pattern']['dimension']['y'])
        image_points_r = np.ones((n_points, 2), np.float32)
        image_points_l = np.ones((n_points, 2), np.float32)

        for idx, point in enumerate(dataset['collections'][collection_key]['labels'][right_camera]['idxs']):
            image_points_r[idx, 0] = point['x']
            image_points_r[idx, 1] = point['y']

        for idx, point in enumerate(dataset['collections'][collection_key]['labels'][left_camera]['idxs']):
            image_points_l[idx, 0] = point['x']
            image_points_l[idx, 1] = point['y']

        imgpoints_l.append(image_points_l)
        imgpoints_r.append(image_points_r)
        objpoints.append(objp)

        # Show detections if desired
        if show_images:
            for point in image_points_l:
                cv2.drawMarker(img_l, tuple(point[0]), (0, 0, 255), cv2.MARKER_CROSS, 14)
                cv2.circle(img_l, tuple(point[0]), 7, (0, 255, 0), lineType=cv2.LINE_AA)

            cv2.imshow(images_left[i], img_l)
            cv2.waitKey(500)

            for point in image_points_r:
                cv2.drawMarker(img_r, tuple(point[0]), (0, 0, 255), cv2.MARKER_CROSS, 14)
                cv2.circle(img_r, tuple(point[0]), 7, (0, 255, 0), lineType=cv2.LINE_AA)

            cv2.imshow(images_right[i], img_l)
            cv2.waitKey(500)

        img_shape = img_l.shape[0:2]

    # Individual intrinsic calibration
    objpoints = np.array(objpoints)
    imgpoints_l = np.array(imgpoints_l)
    imgpoints_r = np.array(imgpoints_r)

    print ('\n---------------------\n> Performing intrinsic calibration for each camera ...')
    ret_l, K_l, D_l, rvecs_l, tvecs_l = cv2.calibrateCamera(objpoints, imgpoints_l, img_shape, None, None)
    ret_r, K_r, D_r, rvecs_r, tvecs_r = cv2.calibrateCamera(objpoints, imgpoints_r, img_shape, None, None)
    print ('>\n---------------------\n Done!\n Starting stereo calibration ...')

    # Extrinsic stereo calibration
    stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                            cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    print (objpoints.shape)
    print (imgpoints_l.shape)
    print (imgpoints_r.shape)
    ret, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_l,
                                                              imgpoints_r, K_l, D_l, K_r,
                                                              D_r, img_shape,
                                                              criteria=stereocalib_criteria)

    print ('>\n---------------------\n Done!\n\n------\nCalibration results:\n------\n')

    print('K_left', K_l)
    print('D_left', D_l)
    print('K_right', K_r)
    print('D_right', D_r)
    print('R', R)
    print('T', T)
    print('E', E)
    print('F', F)

    camera_model = dict([('K_l', K_l), ('K_r', K_r), ('D_l', D_l),
                         ('D_r', D_r), ('rvecs_l', rvecs_l),
                         ('rvecs_r', rvecs_r), ('R', R), ('T', T),
                         ('E', E), ('F', F)])

    cv2.destroyAllWindows()
    return camera_model


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-rc", "--right_camera", help="Right camera sensor name.", type=str, required=True)
    ap.add_argument("-lc", "--left_camera", help="Left camera sensor name.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    left_camera = args['left_camera']
    right_camera = args['right_camera']
    show_images = args['show_images']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Pattern configs
    nx = dataset['calibration_config']['calibration_pattern']['dimension']['x']
    ny = dataset['calibration_config']['calibration_pattern']['dimension']['y']
    square = dataset['calibration_config']['calibration_pattern']['size']
    inner_square = dataset['calibration_config']['calibration_pattern']['inner_size']
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    # Remove partial detections (OpenCV does not support them)
    number_of_corners = int(dataset['calibration_config']['calibration_pattern']['dimension']['x']) * \
                        int(dataset['calibration_config']['calibration_pattern']['dimension']['y'])
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if sensor['msg_type'] == 'Image' and collection['labels'][sensor_key]['detected']:
                if not len(collection['labels'][sensor_key]['idxs']) == number_of_corners:
                    print(
                            Fore.RED + 'Partial detection removed:' + Style.RESET_ALL + ' label from collection ' +
                            collection_key + ', sensor ' + sensor_key)
                    collection['labels'][sensor_key]['detected'] = False

    paths_r = []
    paths_l = []
    od = OrderedDict(sorted(dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        if collection['labels'][right_camera]['detected'] == False \
                or collection['labels'][left_camera]['detected'] == False:
            continue
        # Read image data
        path_r = os.path.dirname(json_file) + '/' + collection['data'][right_camera]['data_file']
        path_l = os.path.dirname(json_file) + '/' + collection['data'][left_camera]['data_file']

        paths_r.append(path_r)
        paths_l.append(path_l)

    cvStereoCalibrate(objp, paths_r, paths_l)
