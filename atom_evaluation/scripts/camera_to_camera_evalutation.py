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

import atom_core.atom
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore

# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-train_json", "--train_json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-test_json", "--test_json_file", help="Json file containing test input dataset.", type=str,
                    required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    show_images = args['show_images']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from files
    # ---------------------------------------
    # Loads the train json file containing the calibration results
    train_json_file = args['train_json_file']
    f = open(train_json_file, 'r')
    train_dataset = json.load(f)
    # Loads the test json file containing a set of collections to evaluate the calibration
    test_json_file = args['test_json_file']
    f = open(test_json_file, 'r')
    test_dataset = json.load(f)

    # ---------------------------------------
    # --- Get intrinsic data for both sensors
    # ---------------------------------------
    # Source sensor
    K_s = np.zeros((3, 3), np.float32);
    D_s = np.zeros((5, 1), np.float32)
    K_s[0, :] = train_dataset['sensors'][source_sensor]['camera_info']['K'][0:3]
    K_s[1, :] = train_dataset['sensors'][source_sensor]['camera_info']['K'][3:6]
    K_s[2, :] = train_dataset['sensors'][source_sensor]['camera_info']['K'][6:9]
    D_s[:, 0] = train_dataset['sensors'][source_sensor]['camera_info']['D'][0:5]

    # Target sensor
    K_t = np.zeros((3, 3), np.float32);
    D_t = np.zeros((5, 1), np.float32)
    K_t[0, :] = train_dataset['sensors'][target_sensor]['camera_info']['K'][0:3]
    K_t[1, :] = train_dataset['sensors'][target_sensor]['camera_info']['K'][3:6]
    K_t[2, :] = train_dataset['sensors'][target_sensor]['camera_info']['K'][6:9]
    D_t[:, 0] = train_dataset['sensors'][target_sensor]['camera_info']['D'][0:5]

    # ---------------------------------------
    # --- Evaluation loop
    # ---------------------------------------
    for collection_key, collection in test_dataset['collections'].items():
        # Get pattern number of corners
        nx = test_dataset['calibration_config']['calibration_pattern']['dimension']['x']
        ny = test_dataset['calibration_config']['calibration_pattern']['dimension']['y']
        square = test_dataset['calibration_config']['calibration_pattern']['size']

        # Get corners on both images
        corners_s = np.zeros((len(collection['labels'][source_sensor]['idxs']), 1, 2), dtype=np.float)
        ids_s = range(0, len(collection['labels'][source_sensor]['idxs']))
        for idx, point in enumerate(collection['labels'][source_sensor]['idxs']):
            corners_s[idx, 0, 0] = point['x']
            corners_s[idx, 0, 1] = point['y']
            ids_s[idx] = point['id']
        # ----
        corners_t = np.zeros((len(collection['labels'][target_sensor]['idxs']), 1, 2), dtype=np.float)
        ids_t = range(0, len(collection['labels'][target_sensor]['idxs']))
        for idx, point in enumerate(collection['labels'][target_sensor]['idxs']):
            corners_t[idx, 0, 0] = point['x']
            corners_t[idx, 0, 1] = point['y']
            ids_t[idx] = point['id']
        # ----

        # Read image data
        path_s = os.path.dirname(test_json_file) + '/' + collection['data'][source_sensor]['data_file']
        path_t = os.path.dirname(test_json_file) + '/' + collection['data'][target_sensor]['data_file']

        image_s = cv2.imread(path_s)
        gray_s = cv2.cvtColor(image_s, cv2.COLOR_BGR2GRAY)
        image_t = cv2.imread(path_t)
        gray_t = cv2.cvtColor(image_t, cv2.COLOR_BGR2GRAY)

        # Show corners on images
        if show_images == True:
            for idx in range(0, len(corners_s)):
                cv2.circle(image_s, (int(corners_s[idx, 0, 0]), int(corners_s[idx, 0, 1])), 3, (0, 0, 255), -1)
            for idx in range(0, len(corners_t)):
                cv2.circle(image_t, (int(corners_t[idx, 0, 0]), int(corners_t[idx, 0, 1])), 3, (0, 0, 255), -1)

            cv2.imshow('source_image', image_s)
            cv2.imshow('target_image', image_t)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # Refine corners estimation
        corners_s = np.array(corners_s, dtype=np.float32)
        corners_t = np.array(corners_t, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.0001)
        corners_s = cv2.cornerSubPix(gray_s, corners_s, (3, 3), (-1, -1), criteria)
        corners_t = cv2.cornerSubPix(gray_t, corners_t, (3, 3), (-1, -1), criteria)

        # Find pose of the source camera w.r.t the pattern
        objp = np.zeros((nx * ny, 3), np.float32)
        objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        ret, rvecs, tvecs = cv2.solvePnP(objp[ids_s], np.array(corners_s, dtype=np.float32), K_s, D_s)
