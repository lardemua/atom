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
        params = cv2.aruco.DetectorParameters_create()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        board = cv2.aruco.CharucoBoard_create(nx + 1, ny + 1, square, inner_square, dictionary)

        corners_l, ids_l, rejected_l = cv2.aruco.detectMarkers(gray_l, dictionary, parameters=params)
        corners_r, ids_r, rejected_r = cv2.aruco.detectMarkers(gray_r, dictionary, parameters=params)

        if len(corners_l) == 0 or len(corners_r) == 0:
            continue

        ret_l, ccorners_l, cids_l = cv2.aruco.interpolateCornersCharuco(corners_l, ids_l, gray_l, board)
        ret_r, ccorners_r, cids_r = cv2.aruco.interpolateCornersCharuco(corners_r, ids_r, gray_r, board)

        imgpoints_l.append(ccorners_l)
        imgpoints_r.append(ccorners_r)
        allids_l.append(cids_l)
        allids_r.append(cids_r)
        objpoints.append(objp)

        # Show detections if desired
        if show_images:
            for point in ccorners_l:
                cv2.drawMarker(img_l, tuple(point[0]), (0, 0, 255), cv2.MARKER_CROSS, 14)
                cv2.circle(img_l, tuple(point[0]), 7, (0, 255, 0), lineType=cv2.LINE_AA)

            cv2.imshow(images_left[i], img_l)
            cv2.waitKey(500)

            for point in ccorners_r:
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
    ret_l, K_l, D_l, rvecs_l, tvecs_l = cv2.aruco.calibrateCameraCharuco(imgpoints_l, allids_l, board, img_shape, None,
                                                                         None)
    ret_r, K_r, D_r, rvecs_r, tvecs_r = cv2.aruco.calibrateCameraCharuco(imgpoints_r, allids_r, board, img_shape, None,
                                                                         None)
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
    ap.add_argument("-train_json", "--train_json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    train_json_file = args['train_json_file']
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    show_images = args['show_images']

    # Read json file
    f = open(train_json_file, 'r')
    train_dataset = json.load(f)

    # Pattern configs
    nx = train_dataset['calibration_config']['calibration_pattern']['dimension']['x']
    ny = train_dataset['calibration_config']['calibration_pattern']['dimension']['y']
    square = train_dataset['calibration_config']['calibration_pattern']['size']
    inner_square = train_dataset['calibration_config']['calibration_pattern']['inner_size']
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    paths_s = []
    paths_t = []
    od = OrderedDict(sorted(train_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # Read image data
        path_s = os.path.dirname(train_json_file) + '/' + collection['data'][source_sensor]['data_file']
        path_t = os.path.dirname(train_json_file) + '/' + collection['data'][target_sensor]['data_file']

        paths_s.append(path_s)
        paths_t.append(path_t)

    cvStereoCalibrate(objp, paths_s, paths_t)
