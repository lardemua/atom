#!/usr/bin/env python3

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import json
import os
import numpy as np
import ros_numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import atom_core.atom
from atom_core.dataset_io import getPointCloudMessageFromDictionary, read_pcd

from rospy_message_converter import message_converter
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore
from collections import OrderedDict
from image_geometry import PinholeCameraModel

from atom_core.naming import generateKey
from atom_calibration.collect.label_messages import convertDepthImage32FC1to16UC1, convertDepthImage16UC1to32FC1
from scipy.spatial.distance import directed_hausdorff


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def rangeToImage(collection, json_file, ss):
    filename = os.path.dirname(json_file) + '/' + collection['data'][ss]['data_file']
    msg = read_pcd(filename)
    collection['data'][ss].update(message_converter.convert_ros_message_to_dictionary(msg))

    cloud_msg = getPointCloudMessageFromDictionary(collection['data'][ss])
    # idxs = collection['labels'][ss]['idxs_limit_points']
    idxs = collection['labels'][ss]['idxs']

    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points_in_vel = np.zeros((4, pc.shape[0]))
    points_in_vel[0, :] = pc['x']
    points_in_vel[1, :] = pc['y']
    points_in_vel[2, :] = pc['z']
    points_in_vel[3, :] = 1

    # points_in_cam = np.dot(tf, points_in_vel)

    # # -- Project them to the image
    # w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    # K = np.ndarray((3, 3), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    # D = np.ndarray((5, 1), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)
    #
    # lidar_pts_in_img, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return points_in_vel


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-train_json", "--train_json_file", help="Json file containing input training dataset.", type=str,
                    required=True)
    ap.add_argument("-test_json", "--test_json_file", help="Json file containing input testing dataset.", type=str,
                    required=True)
    ap.add_argument("-ld1", "--lidar_sensor_1", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ld2", "--lidar_sensor_2", help="Target transformation sensor.", type=str, required=True)
    # ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    lidar_sensor_1 = args['lidar_sensor_1']
    lidar_sensor_2 = args['lidar_sensor_2']
    # show_images = args['show_images']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    train_json_file = args['train_json_file']
    f = open(train_json_file, 'r')
    train_dataset = json.load(f)
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
            test_dataset['sensors'][train_sensor_key]['camera_info']['D'] = train_sensor['camera_info']['D']
            test_dataset['sensors'][train_sensor_key]['camera_info']['K'] = train_sensor['camera_info']['K']
            test_dataset['sensors'][train_sensor_key]['camera_info']['P'] = train_sensor['camera_info']['P']
            test_dataset['sensors'][train_sensor_key]['camera_info']['R'] = train_sensor['camera_info']['R']

    # ---------------------------------------
    # --- INITIALIZATION Read evaluation data from file ---> if desired <---
    # ---------------------------------------
    print(Fore.BLUE + '\nStarting evalutation...')
    print(Fore.WHITE)
    print(
        '-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print(
        '{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}'.format('#', 'RMS', 'Avg Error', 'X Error', 'Y Error',
                                                                   'X Standard Deviation',
                                                                   'Y Standard Deviation'))
    print(
        '-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')

    # Declare output dict to save the evaluation data if desire
    delta_total = []
    output_dict = {}
    output_dict['ground_truth_pts'] = {}

    from_frame = test_dataset['calibration_config']['sensors'][lidar_sensor_1]['link']
    to_frame = test_dataset['calibration_config']['sensors'][lidar_sensor_2]['link']
    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # if not collection_key=='2':
        #     continue
        # if show_images:
        #     fig = plt.figure()
        #     ax = fig.add_subplot(projection='3d')
            # ax.xlim([-5,5])
            # ax.ylim([-5, 5])
            # ax.zlim([-5, 5])

        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        lidar1_to_lidar2 = atom_core.atom.getTransform(to_frame, from_frame,
                                                       test_dataset['collections'][collection_key]['transforms'])
        lidar_pts_1 = rangeToImage(collection, test_json_file, lidar_sensor_1)
        lidar_pts_2 = rangeToImage(collection, test_json_file, lidar_sensor_2)
        lidar_pts_1_in_lidar_2 = np.dot(lidar1_to_lidar2, lidar_pts_1)
        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        delta_pts = []
        distances = []
        for idx in range(lidar_pts_2.shape[1]):
            lidar_pt = np.reshape(lidar_pts_2[0:3, idx], (1, 3))
            dist = distance.cdist(lidar_pt, lidar_pts_1_in_lidar_2.transpose()[:, :3], 'euclidean')
            delta_pts.append((np.min(dist)))
            coords = np.where(dist == np.min(dist))

            min_dist_pt = lidar_pts_1_in_lidar_2.transpose()[coords[1]][0,:3]
            dist = abs(lidar_pt - min_dist_pt)
            distances.append(dist)
            delta_total.append(dist)

            # if show_images:
            #     ax.plot((lidar_pt[0,0],min_dist_pt[0]),(lidar_pt[0,1],min_dist_pt[1]),(lidar_pt[0,2],min_dist_pt[2]), c='y' )

        if len(delta_pts) == 0:
            print('No LiDAR point mapped into the image for collection ' + str(collection_key))
            continue

        # ---------------------------------------
        # --- Compute error metrics
        # ---------------------------------------
        total_pts = len(delta_pts)
        delta_pts = np.array(delta_pts, np.float32)
        avg_error = np.sum(np.abs(delta_pts)) / total_pts
        rms = np.sqrt((delta_pts ** 2).mean())

        delta_xy = np.array(distances, np.float32)
        delta_xy = delta_xy[:, 0]
        avg_error_x = np.sum(np.abs(delta_xy[:, 0])) / total_pts
        avg_error_y = np.sum(np.abs(delta_xy[:, 1])) / total_pts
        stdev_xy = np.std(delta_xy, axis=0)

        print(
            '{:^25s}{:^25f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format(collection_key, rms,
                                                                                 avg_error, avg_error_x,
                                                                                 avg_error_y,
                                                                                 stdev_xy[0], stdev_xy[1]))

        # ---------------------------------------
        # --- Drawing ...
        # ---------------------------------------
        # if show_images:
        #     for idx in range(0, lidar_pts_1_in_lidar_2.shape[1]):
        #         ax.scatter(lidar_pts_1_in_lidar_2[0, idx], lidar_pts_1_in_lidar_2[1, idx], lidar_pts_1_in_lidar_2[2,idx], c='r')
        #     for idx in range(0, lidar_pts_2.shape[1]):
        #         ax.scatter(lidar_pts_2[0, idx], lidar_pts_2[1, idx],lidar_pts_2[2, idx], c='b')
        #     plt.show()

    total_pts = len(delta_total)
    delta_total = np.array(delta_total, np.float32)
    avg_error = np.sum(np.abs(delta_total)) / total_pts
    rms = np.sqrt((delta_total ** 2).mean())

    delta_xy = np.array(delta_total, np.float32)
    delta_xy = delta_xy[:, 0]
    avg_error_x = np.sum(np.abs(delta_xy[:, 0])) / total_pts
    avg_error_y = np.sum(np.abs(delta_xy[:, 1])) / total_pts
    stdev_xy = np.std(delta_xy, axis=0)

    print(
        '-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print(
        '{:^25s}{:^25f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format('All', rms,
                                                                             avg_error, avg_error_x,
                                                                             avg_error_y,
                                                                             stdev_xy[0], stdev_xy[1]))
    print(
        '-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print("Press ESC to quit and close all open windows.")

    # plt.waitforbuttonpress(0)  # this will wait for indefinite time
    # plt.close(fig)

    # while True:
    #     k = cv2.waitKey(0) & 0xFF
    #     if k == 27:
    #         fig.close()
    #         break
# Save evaluation data
# if use_annotation is True:
#     createJSONFile(eval_file, output_dict)
