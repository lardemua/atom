#!/usr/bin/env python

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import json
import os
import numpy as np
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def rangeToImage(collection, ss, ts, tf):
    # -- Convert limit points from velodyne to camera frame
    pts = collection['labels'][ss]['limit_points']
    points_in_vel = np.array(
        [[item['x'] for item in pts], [item['y'] for item in pts], [item['z'] for item in pts],
         [1 for item in pts]], np.float)

    points_in_cam = np.dot(tf, points_in_vel)

    # -- Project them to the image
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)

    pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image


def getPatternCorners(dataset, collection, ts):
    # - Get pattern dimensions
    nx = dataset['calibration_config']['calibration_pattern']['dimension']['x']
    ny = dataset['calibration_config']['calibration_pattern']['dimension']['y']
    square = dataset['calibration_config']['calibration_pattern']['size']
    if type(dataset['calibration_config']['calibration_pattern']['border_size']) is dict:
        border_x = dataset['calibration_config']['calibration_pattern']['border_size']['x']
        border_y = dataset['calibration_config']['calibration_pattern']['border_size']['y']
    else:
        border_x = border_y = dataset['calibration_config']['calibration_pattern']['border_size']

    # - Get pattern detected corners
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    corners = np.zeros((len(collection['labels'][ts]['idxs']), 1, 2), dtype=np.float)
    ids = range(0, len(collection['labels'][ts]['idxs']))
    for idx, point in enumerate(collection['labels'][ts]['idxs']):
        corners[idx, 0, 0] = point['x']
        corners[idx, 0, 1] = point['y']
        ids[idx] = point['id']

    # - Sensor to pattern transformation
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)

    ret, rvecs, tvecs = cv2.solvePnP(objp[ids], np.array(corners, dtype=np.float32), K, D)
    sensor_to_pattern = opt_utilities.traslationRodriguesToTransform(tvecs, rvecs)

    # - Pattern extrema corners to image
    top_left = np.array([-square - border_x, -square - border_y, 0, 1])
    top_right = np.array([nx * square + border_x, -square - border_y, 0, 1])
    bottom_right = np.array([nx * square + border_x, ny * square + border_y, 0, 1])
    bottom_left = np.array([-square - border_x, ny * square + border_y, 0, 1])

    top_left_on_cam = np.dot(sensor_to_pattern, np.transpose(top_left))
    top_right_on_cam = np.dot(sensor_to_pattern, np.transpose(top_right))
    bottom_left_on_cam = np.dot(sensor_to_pattern, np.transpose(bottom_left))
    bottom_right_on_cam = np.dot(sensor_to_pattern, np.transpose(bottom_right))

    corners_on_cam = np.zeros((4, 4), dtype=np.float32)
    corners_on_cam[:, 0] = top_left_on_cam
    corners_on_cam[:, 1] = top_right_on_cam
    corners_on_cam[:, 2] = bottom_left_on_cam
    corners_on_cam[:, 3] = bottom_right_on_cam

    corners_on_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, corners_on_cam[0:3, :])

    return corners_on_image


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="shows images for each camera", action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    show_images = args['show_images']

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    from_frame = dataset['calibration_config']['sensors'][target_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][source_sensor]['link']
    for collection_key, collection in dataset['collections'].items():
        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        vel2cam = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
        pts_in_image = rangeToImage(collection, source_sensor, target_sensor, vel2cam)

        # ---------------------------------------
        # --- Get pattern corners
        # ---------------------------------------
        corners_on_image = getPatternCorners(dataset, collection, target_sensor)

        # ---------------------------------------
        # --- Drawing ...
        # ---------------------------------------
        if show_images == True:
            filename = os.path.dirname(args['json_file']) + '/' + collection['data'][target_sensor]['data_file']
            image = cv2.imread(filename)

            for idx in range(0, pts_in_image.shape[1]):
                image = cv2.circle(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])), 5, (255, 0, 0), -1)
            for idx in range(0, corners_on_image.shape[1]):
                image = cv2.circle(image, (int(corners_on_image[0, idx]), int(corners_on_image[1, idx])), 5,
                                   (0, 255, 0), -1)

            cv2.imshow("LiDAR Reprojection", image)
            cv2.waitKey()
