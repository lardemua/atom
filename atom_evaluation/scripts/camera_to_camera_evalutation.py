#!/usr/bin/env python

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import json
import math
import os
import numpy as np
import atom_core.atom
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore
from numpy.linalg import inv
from matplotlib import cm


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def computeHomographyMat(collection, rvecs, tvecs, K_s, D_s, K_t, D_t):
    # ----------------------------------------------------------------------
    # ----- Find pose of the source and target cameras w.r.t the pattern
    # ----------------------------------------------------------------------
    ss_T_chess_h = np.zeros((4, 4), np.float32)
    ss_T_chess_h[3, 3] = 1
    ss_T_chess_h[0:3, 3] = tvecs[:, 0]
    ss_T_chess_h[0:3, 0:3] = opt_utilities.rodriguesToMatrix(rvecs)

    target_frame = train_dataset['calibration_config']['sensors'][target_sensor]['link']
    source_frame = train_dataset['calibration_config']['sensors'][source_sensor]['link']

    selected_collection_key = train_dataset['collections'].keys()[0]

    st_T_ss = atom_core.atom.getTransform(target_frame, source_frame,
                                          train_dataset['collections'][selected_collection_key]['transforms'])
    ss_T_st = atom_core.atom.getTransform(source_frame, target_frame,
                                          train_dataset['collections'][selected_collection_key]['transforms'])

    st_T_chess_h = np.dot(inv(ss_T_st), ss_T_chess_h)

    ss_T_chess = np.zeros((3, 3), np.float32)
    st_T_chess = np.zeros((3, 3), np.float32)

    for c in range(0, 2):
        for l in range(0, 3):
            ss_T_chess[l, c] = ss_T_chess_h[l, c]
            st_T_chess[l, c] = st_T_chess_h[l, c]

    ss_T_chess[:, 2] = ss_T_chess_h[0:3, 3]
    st_T_chess[:, 2] = st_T_chess_h[0:3, 3]

    # ---------------------------------------------------------------------
    # ----- Compute homography matrix
    # ---------------------------------------------------------------------
    A = np.dot(K_t, st_T_chess)
    B = np.dot(A, inv(ss_T_chess))
    C = np.dot(B, inv(K_s))
    H = C

    return H


def undistortCorners(corners, K, D):
    # from https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    # where it says x'' = ... x'o

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = D

    # compute the homogeneous image coordinates (non pixels)
    w = corners.shape[0]
    homogenous_corners = np.ones((3, w), np.float32)
    homogenous_corners[0, :] = (corners[:, 0] - cx) / fx
    homogenous_corners[1, :] = (corners[:, 1] - cy) / fy

    # apply undistortion
    xl = homogenous_corners[0, :]
    yl = homogenous_corners[1, :]
    r2 = xl ** 2 + yl ** 2  # r square (used multiple times bellow)
    xll = xl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + 2 * p1 * xl * yl + p2 * (r2 + 2 * xl ** 2)
    yll = yl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + p1 * (r2 + 2 * yl ** 2) + 2 * p2 * xl * yl

    # recompute pixels (now undistorted)
    undistorted_corners = np.ones((3, w), np.float32)
    undistorted_corners[0, :] = xll * fx + cx
    undistorted_corners[1, :] = yll * fy + cy
    return undistorted_corners


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
    print(Fore.GREEN + '\nStarting evalutation...')
    print(Fore.GREEN + 'If you enabled the visualization mode - press [SPACE] to advance between images\n')
    print(Fore.WHITE)
    print(
        '---------------------------------------------------------------------------------------------------------------------------------')
    print('{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}'.format('#', 'X Error', 'Y Error', 'X Standard Deviation',
                                                       'Y Standard Deviation'))
    print(
        '---------------------------------------------------------------------------------------------------------------------------------')
    for collection_key, collection in test_dataset['collections'].items():
        # Get pattern number of corners
        nx = test_dataset['calibration_config']['calibration_pattern']['dimension']['x']
        ny = test_dataset['calibration_config']['calibration_pattern']['dimension']['y']
        square = test_dataset['calibration_config']['calibration_pattern']['size']

        # Get corners on both images
        corners_s = np.zeros((len(collection['labels'][source_sensor]['idxs']), 2), dtype=np.float)
        idxs_s = range(0, len(collection['labels'][source_sensor]['idxs']))
        for idx, point in enumerate(collection['labels'][source_sensor]['idxs']):
            corners_s[idx, 0] = point['x']
            corners_s[idx, 1] = point['y']
            idxs_s[idx] = point['id']
        # ----
        corners_t = np.zeros((len(collection['labels'][target_sensor]['idxs']), 2), dtype=np.float)
        idxs_t = range(0, len(collection['labels'][target_sensor]['idxs']))
        for idx, point in enumerate(collection['labels'][target_sensor]['idxs']):
            corners_t[idx, 0] = point['x']
            corners_t[idx, 1] = point['y']
            idxs_t[idx] = point['id']
        # ----

        # Read image data
        path_s = os.path.dirname(test_json_file) + '/' + collection['data'][source_sensor]['data_file']
        path_t = os.path.dirname(test_json_file) + '/' + collection['data'][target_sensor]['data_file']

        image_s = cv2.imread(path_s)
        gray_s = cv2.cvtColor(image_s, cv2.COLOR_BGR2GRAY)
        image_t = cv2.imread(path_t)
        gray_t = cv2.cvtColor(image_t, cv2.COLOR_BGR2GRAY)

        # Refine corners estimation
        corners_s = np.array(corners_s, dtype=np.float32)
        corners_t = np.array(corners_t, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.0001)
        corners_s = cv2.cornerSubPix(gray_s, corners_s, (3, 3), (-1, -1), criteria)
        corners_t = cv2.cornerSubPix(gray_t, corners_t, (3, 3), (-1, -1), criteria)

        # Compute camera pose w.r.t the pattern
        objp = np.zeros((nx * ny, 3), np.float32)
        objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        ret, rvecs, tvecs = cv2.solvePnP(objp[idxs_s], np.array(corners_s, dtype=np.float32), K_s, D_s)

        # Get homography matrix
        if not ret:
            print("ERROR: Pattern wasn't found on collection " + str(collection_key))
            continue
        else:
            H = computeHomographyMat(collection, rvecs, tvecs, K_s, D_s, K_t, D_t)

        # Project corners of source image into target image
        corners_t_proj = np.dot(H, undistortCorners(corners_s, K_s, D_s))
        for i in range(0, corners_t_proj.shape[1]):
            corners_t_proj[0, i] = corners_t_proj[0, i] / corners_t_proj[2, i]
            corners_t_proj[1, i] = corners_t_proj[1, i] / corners_t_proj[2, i]

        # Show projection
        if show_images == True:
            width = collection['data'][target_sensor]['width']
            height = collection['data'][target_sensor]['height']
            diagonal = math.sqrt(width ** 2 + height ** 2)

            cmap = cm.gist_rainbow(np.linspace(0, 1, nx * ny))
            cmap = cm.tab20b(np.linspace(0, 1, len(corners_t)))
            for idx in range(0, len(corners_t)):
                x = int(corners_t[idx, 0])
                y = int(corners_t[idx, 1])
                color = (cmap[idx, 2] * 255, cmap[idx, 1] * 255, cmap[idx, 0] * 255)
                opt_utilities.drawSquare2D(image_t, x, y, int(8E-3 * diagonal), color=color, thickness=1)

            cmap = cm.gist_rainbow(np.linspace(0, 1, nx * ny))
            cmap = cm.tab20b(np.linspace(0, 1, corners_t_proj.shape[1]))
            for idx in range(0, corners_t_proj.shape[1]):
                x = int(corners_t_proj[0, idx])
                y = int(corners_t_proj[1, idx])
                color = (cmap[idx, 2] * 255, cmap[idx, 1] * 255, cmap[idx, 0] * 255)
                opt_utilities.drawCross2D(image_t, x, y, int(8E-3 * diagonal), color=color, thickness=1)

            cv2.imshow('Reprojection error', image_t)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # Compute reprojection error
        delta_pts = []
        for idx_t in idxs_t:
            if idx_t in idxs_s:
                array_idx_t = idxs_t.index(idx_t)
                array_idx_s = idxs_s.index(idx_t)

                diff = corners_t_proj[0:2, array_idx_s] - undistortCorners(corners_t, K_t, D_t)[0:2, array_idx_t]
                delta_pts.append(diff)

        total_pts = len(delta_pts)
        delta_pts = np.array(delta_pts, np.float32)
        avg_error_x = np.sum(np.abs(delta_pts[:, 0])) / total_pts
        avg_error_y = np.sum(np.abs(delta_pts[:, 1])) / total_pts
        stdev = np.std(delta_pts, axis=0)

        # Print error metrics
        print('{:^25s}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format(collection_key, avg_error_x, avg_error_y, stdev[0],
                                                                   stdev[1]))