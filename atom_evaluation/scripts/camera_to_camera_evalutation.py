#!/usr/bin/env python3

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
import matplotlib.pyplot as plt
import atom_core.atom
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore
from numpy.linalg import inv
from matplotlib import cm

from collections import OrderedDict

from atom_core.naming import generateKey


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def computeHomographyMat(collection, collection_key, rvecs, tvecs, K_s, D_s, K_t, D_t):
    # ----------------------------------------------------------------------
    # ----- Find pose of the source and target cameras w.r.t the pattern
    # ----------------------------------------------------------------------
    ss_T_chess_h = np.zeros((4, 4), np.float32)
    ss_T_chess_h[3, 3] = 1
    ss_T_chess_h[0:3, 3] = tvecs[:, 0]
    ss_T_chess_h[0:3, 0:3] = opt_utilities.rodriguesToMatrix(rvecs)

    target_frame = test_dataset['calibration_config']['sensors'][target_sensor]['link']
    source_frame = test_dataset['calibration_config']['sensors'][source_sensor]['link']

    selected_collection_key = list(test_dataset['collections'].keys())[0]

    st_T_ss = atom_core.atom.getTransform(target_frame, source_frame,
                                          test_dataset['collections'][collection_key]['transforms'])
    ss_T_st = atom_core.atom.getTransform(source_frame, target_frame,
                                          test_dataset['collections'][collection_key]['transforms'])

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

    return H, np.dot(st_T_ss, ss_T_chess_h)


def undistortCorners(corners, K, D):
    """ Remove distortion from corner points. """

    points = cv2.undistortPoints(corners, K, D);

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    undistorted_corners = np.ones((3, corners.shape[0]), np.float32)
    undistorted_corners[0, :] = points[:, 0, 0] * fx + cx
    undistorted_corners[1, :] = points[:, 0, 1] * fy + cy

    return undistorted_corners


def distortCorners(corners, K, D):
    # from https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    # where it says x'' = ... x'o

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = D

    # # compute the homogeneous image coordinates (non pixels)
    xl = (corners[:, 0] - cx) / fx
    yl = (corners[:, 1] - cy) / fy

    # # apply undistortion
    r2 = xl ** 2 + yl ** 2  # r square (used multiple times bellow)
    xll = xl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + 2 * p1 * xl * yl + p2 * (r2 + 2 * xl ** 2)
    yll = yl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + p1 * (r2 + 2 * yl ** 2) + 2 * p2 * xl * yl

    distorted_corners = np.ones((2, corners.shape[0]), np.float32)
    distorted_corners[0, :] = xll * fx + cx
    distorted_corners[1, :] = yll * fy + cy

    return distorted_corners


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
    ap.add_argument("-po", "--pattern_object", help="Use pattern object projection instead of Homography.",
                    action='store_true', default=False)
    ap.add_argument("-sg", "--save_graphics", help="Save reprojection error graphics.",
                    action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    show_images = args['show_images']
    use_pattern_object = args['pattern_object']
    save_graphics = args['save_graphics']

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
    # --- Get mixed json (calibrated transforms from train and the rest from test)
    # ---------------------------------------
    # test_dataset = test_dataset
    # I am just using the test dataset for everything. If we need an original test dataset we can copy here.

    # Replace optimized transformations in the test dataset copying from the train dataset
    for sensor_key, sensor in train_dataset['sensors'].items():
        calibration_parent = sensor['calibration_parent']
        calibration_child = sensor['calibration_child']
        transform_name = generateKey(calibration_parent, calibration_child)

        # We can only optimized fixed transformations, so the optimized transform should be the same for all
        # collections. We select the first collection (selected_collection_key) and retrieve the optimized
        # transformation for that.
        selected_collection_key = list(train_dataset['collections'].keys())[0]
        optimized_transform = train_dataset['collections'][selected_collection_key]['transforms'][transform_name]

        # iterate all collections of the test dataset and replace the optimized transformation
        for collection_key, collection in test_dataset['collections'].items():
            collection['transforms'][transform_name]['quat'] = optimized_transform['quat']
            collection['transforms'][transform_name]['trans'] = optimized_transform['trans']

    # Copy intrinsic parameters for cameras from train to test dataset.
    for train_sensor_key, train_sensor in train_dataset['sensors'].items():
        if train_sensor['msg_type'] == 'Image':
            train_dataset['sensors'][train_sensor_key]['camera_info']['D'] = train_sensor['camera_info']['D']
            train_dataset['sensors'][train_sensor_key]['camera_info']['K'] = train_sensor['camera_info']['K']
            train_dataset['sensors'][train_sensor_key]['camera_info']['P'] = train_sensor['camera_info']['P']
            train_dataset['sensors'][train_sensor_key]['camera_info']['R'] = train_sensor['camera_info']['R']

    f = open('test.json', 'w')
    json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
    print (json.dumps(test_dataset, indent=2, sort_keys=True), file=f)
    f.close()
    # exit(0)

    # ---------------------------------------
    # --- Get intrinsic data for both sensors
    # ---------------------------------------
    # Source sensor
    K_s = np.zeros((3, 3), np.float32)
    D_s = np.zeros((5, 1), np.float32)
    K_s[0, :] = test_dataset['sensors'][source_sensor]['camera_info']['K'][0:3]
    K_s[1, :] = test_dataset['sensors'][source_sensor]['camera_info']['K'][3:6]
    K_s[2, :] = test_dataset['sensors'][source_sensor]['camera_info']['K'][6:9]
    D_s[:, 0] = test_dataset['sensors'][source_sensor]['camera_info']['D'][0:5]

    # Target sensor
    K_t = np.zeros((3, 3), np.float32)
    D_t = np.zeros((5, 1), np.float32)
    K_t[0, :] = test_dataset['sensors'][target_sensor]['camera_info']['K'][0:3]
    K_t[1, :] = test_dataset['sensors'][target_sensor]['camera_info']['K'][3:6]
    K_t[2, :] = test_dataset['sensors'][target_sensor]['camera_info']['K'][6:9]
    D_t[:, 0] = test_dataset['sensors'][target_sensor]['camera_info']['D'][0:5]

    # ---------------------------------------
    # --- Evaluation loop
    # ---------------------------------------
    print(Fore.GREEN + '\nStarting evaluation...')
    print(Fore.GREEN + 'If you enabled the visualization mode - press [SPACE] to advance between images')
    print(Fore.GREEN + '                                      - to stop visualization press [c] or [q]\n')
    print(Fore.WHITE)
    print('---------------------------------------------------------------------------')
    print('{:^5s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}'.format(
        '#', 'RMS', 'X err', 'Y err', 'X std', 'Y std', 'T err', 'R err'))
    print('---------------------------------------------------------------------------')

    delta_total = []
    terr = []
    rerr = []

    # Deleting collections where the pattern is not found by all sensors:
    collections_to_delete = []
    for collection_key, collection in test_dataset['collections'].items():
        for sensor_key, sensor in test_dataset['sensors'].items():
            if not collection['labels'][sensor_key]['detected'] and (
                    sensor_key == source_sensor or sensor_key == target_sensor):
                print(
                        Fore.RED + "Removing collection " + collection_key + ' -> pattern was not found in sensor ' +
                        sensor_key + ' (must be found in all sensors).' + Style.RESET_ALL)

                collections_to_delete.append(collection_key)
                break

    for collection_key in collections_to_delete:
        del test_dataset['collections'][collection_key]

    # Reprojection error graphics definitions
    colors = cm.tab20b(np.linspace(0, 1, len(test_dataset['collections'].items())))

    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # Get pattern number of corners
        nx = test_dataset['calibration_config']['calibration_pattern']['dimension']['x']
        ny = test_dataset['calibration_config']['calibration_pattern']['dimension']['y']
        square = test_dataset['calibration_config']['calibration_pattern']['size']

        # Get corners on both images
        corners_s = np.zeros((len(collection['labels'][source_sensor]['idxs']), 2), dtype=np.float)
        idxs_s = list(range(0, len(collection['labels'][source_sensor]['idxs'])))
        for idx, point in enumerate(collection['labels'][source_sensor]['idxs']):
            corners_s[idx, 0] = point['x']
            corners_s[idx, 1] = point['y']
            idxs_s[idx] = point['id']
        # ----
        corners_t = np.zeros((len(collection['labels'][target_sensor]['idxs']), 2), dtype=np.float)
        idxs_t = list(range(0, len(collection['labels'][target_sensor]['idxs'])))
        for idx, point in enumerate(collection['labels'][target_sensor]['idxs']):
            corners_t[idx, 0] = point['x']
            corners_t[idx, 1] = point['y']
            idxs_t[idx] = point['id']
        # ----

        # Read image data
        # TODO This is only needed if we have flag show images right?
        path_s = os.path.dirname(test_json_file) + '/' + collection['data'][source_sensor]['data_file']
        path_t = os.path.dirname(test_json_file) + '/' + collection['data'][target_sensor]['data_file']

        image_s = cv2.imread(path_s)
        gray_s = cv2.cvtColor(image_s, cv2.COLOR_BGR2GRAY)
        image_t = cv2.imread(path_t)
        gray_t = cv2.cvtColor(image_t, cv2.COLOR_BGR2GRAY)

        # Refine corners estimation
        # TODO Andre, should you really refine? because you should use what the calibration used no? Not sure ...
        corners_s = np.array(corners_s, dtype=np.float32)
        corners_t = np.array(corners_t, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.0001)
        corners_s = cv2.cornerSubPix(gray_s, corners_s, (3, 3), (-1, -1), criteria)
        corners_t = cv2.cornerSubPix(gray_t, corners_t, (3, 3), (-1, -1), criteria)

        # Compute camera pose w.r.t the pattern
        objp = np.zeros((nx * ny, 4), np.float)
        objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
        objp[:, 3] = 1

        # == Compute Translation and Rotation errors
        selected_collection_key = list(test_dataset['collections'].keys())[0]
        common_frame = test_dataset['calibration_config']['world_link']
        target_frame = test_dataset['calibration_config']['sensors'][target_sensor]['link']
        source_frame = test_dataset['calibration_config']['sensors'][source_sensor]['link']

        ret, rvecs, tvecs = cv2.solvePnP(objp.T[:3, :].T[idxs_t], np.array(corners_t, dtype=np.float32), K_t, D_t)
        pattern_pose_target = opt_utilities.traslationRodriguesToTransform(tvecs, rvecs)

        bTp = atom_core.atom.getTransform(common_frame, target_frame,
                                          test_dataset['collections'][collection_key]['transforms'])

        pattern_pose_target = np.dot(bTp, pattern_pose_target)

        # NOTE(eurico): rvecs and tvecs are used ahead to compute the homography
        ret, rvecs, tvecs = cv2.solvePnP(objp.T[:3, :].T[idxs_s], np.array(corners_s, dtype=np.float32), K_s, D_s)
        pattern_pose_source = opt_utilities.traslationRodriguesToTransform(tvecs, rvecs)

        bTp = atom_core.atom.getTransform(common_frame, source_frame,
                                          test_dataset['collections'][collection_key]['transforms'])


        pattern_pose_source = np.dot(bTp, pattern_pose_source)

        delta = np.dot(np.linalg.inv(pattern_pose_source), pattern_pose_target)

        deltaT = delta[0:3, 3]
        deltaR = opt_utilities.matrixToRodrigues(delta[0:3, 0:3])

        terr.append(deltaT * deltaT)
        rerr.append(deltaR * deltaR)
        # ==

        # Get homography matrix
        if not ret:
            print("ERROR: Pattern wasn't found on collection " + str(collection_key))
            continue
        else:
            H, T = computeHomographyMat(collection, collection_key, rvecs, tvecs, K_s, D_s, K_t, D_t)

        # Project corners of source image into target image
        if not use_pattern_object:
            corners_t_proj = np.dot(H, undistortCorners(corners_s, K_s, D_s))
            for i in range(0, corners_t_proj.shape[1]):
                corners_t_proj[0, i] = corners_t_proj[0, i] / corners_t_proj[2, i]
                corners_t_proj[1, i] = corners_t_proj[1, i] / corners_t_proj[2, i]
        else:
            corners_t_proj = np.dot(T, objp.T)
            corners_t_proj, _, _ = opt_utilities.projectToCamera(K_t, D_t, 0, 0, corners_t_proj.T[idxs_s].T)

        # Compute reprojection error
        delta_pts = []
        for idx_t in idxs_t:
            if idx_t in idxs_s:
                array_idx_t = idxs_t.index(idx_t)
                array_idx_s = idxs_s.index(idx_t)

                if not use_pattern_object:
                    diff = corners_t_proj[0:2, array_idx_s] - undistortCorners(corners_t, K_t, D_t)[0:2, array_idx_t]
                else:
                    diff = corners_t_proj[0:2, array_idx_s] - corners_t.T[0:2, array_idx_t]

                delta_pts.append(diff)
                delta_total.append(diff)

                # Compute reprojection error graphics
                plt.plot(diff[0], diff[1], 'o', label=collection_key, alpha=0.7, color=colors[int(collection_key)])


        total_pts = len(delta_pts)
        delta_pts = np.array(delta_pts, np.float32)
        avg_error_x = np.sum(np.abs(delta_pts[:, 0])) / total_pts
        avg_error_y = np.sum(np.abs(delta_pts[:, 1])) / total_pts
        stdev = np.std(delta_pts, axis=0)

        # Print error metrics
        print('{:^5s}{:^10s}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}'.format(
            collection_key, '-', avg_error_x, avg_error_y, stdev[0], stdev[1],
            np.linalg.norm(deltaT) * 1000, np.linalg.norm(deltaR) * 180.0 / np.pi))

        # Show projection
        if show_images:
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

            if not use_pattern_object:
                corners_t_proj = distortCorners(corners_t_proj.T, K_t, D_t)

            for idx in range(0, corners_t_proj.shape[1]):
                x = int(corners_t_proj[0, idx])
                y = int(corners_t_proj[1, idx])
                color = (cmap[idx, 2] * 255, cmap[idx, 1] * 255, cmap[idx, 0] * 255)
                opt_utilities.drawCross2D(image_t, x, y, int(8E-3 * diagonal), color=color, thickness=1)

            cv2.imshow('Reprojection error', image_t)
            key = cv2.waitKey(0)
            if key == ord('c') or key == ord('q'):
                show_images = False
                cv2.destroyAllWindows()

    total_pts = len(delta_total)
    delta_total = np.array(delta_total, np.float)
    avg_error_x = np.sum(np.abs(delta_total[:, 0])) / total_pts
    avg_error_y = np.sum(np.abs(delta_total[:, 1])) / total_pts
    stdev = np.std(delta_total, axis=0)
    rms = np.sqrt((delta_total ** 2).mean())

    if save_graphics:
        axes = plt.gca()
        axes.grid(True)
        plt.tight_layout()
        plt.savefig('rmse-matrix.pdf')

    print('---------------------------------------------------------------------------')
    print('{:^5s}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}{:^10.4f}'.format(
        'All', rms, avg_error_x, avg_error_y, stdev[0], stdev[1],
        np.mean(np.sqrt(np.sum(terr, 1))), #* 1000,
        np.mean(np.sqrt(np.sum(rerr, 1))))) #* 180.0 / np.pi))
    print('---------------------------------------------------------------------------')
