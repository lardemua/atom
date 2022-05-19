#!/usr/bin/env python3

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

# Standard imports
import json
import os
import argparse

from collections import OrderedDict
import sys

import numpy as np
import ros_numpy
import cv2
from scipy.spatial import distance
from colorama import Style, Fore

# ROS imports
from rospy_message_converter import message_converter

# Atom imports
from atom_core.naming import generateKey
from atom_core.vision import projectToCamera
from atom_core.dataset_io import getPointCloudMessageFromDictionary, read_pcd
from atom_core.atom import getTransform

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def rangeToImage(collection, json_file, ss, ts, tf):
    filename = os.path.dirname(json_file) + '/' + collection['data'][ss]['data_file']
    msg = read_pcd(filename)
    collection['data'][ss].update(message_converter.convert_ros_message_to_dictionary(msg))

    cloud_msg = getPointCloudMessageFromDictionary(collection['data'][ss])
    idxs = collection['labels'][ss]['idxs_limit_points']

    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points_in_vel = np.zeros((4, pc.shape[0]))
    points_in_vel[0, :] = pc['x']
    points_in_vel[1, :] = pc['y']
    points_in_vel[2, :] = pc['z']
    points_in_vel[3, :] = 1

    points_in_cam = np.dot(tf, points_in_vel)

    # -- Project them to the image
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['K']), dtype=float)
    D = np.ndarray((5, 1), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['D']), dtype=float)

    pts_in_image, _, _ = projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image

# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-train_json", "--train_json_file", help="Json file containing input training dataset.", type=str,
                    required=True)
    ap.add_argument("-test_json", "--test_json_file", help="Json file containing input testing dataset.", type=str,
                    required=True)
    ap.add_argument("-rs", "--range_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-cs", "--camera_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)
    # ap.add_argument("-ef", "--eval_file", help="Path to file to read and/or write the evalutation data.", type=str,
    #                 required=True)
    # ap.add_argument("-ua", "--use_annotation", help="If true, the limit points will be manually annotated.",
    #                 action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    range_sensor = args['range_sensor']
    camera_sensor = args['camera_sensor']
    show_images = args['show_images']
    # eval_file = args['eval_file']
    # use_annotation = args['use_annotation']

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

    annotation_file = os.path.dirname(test_json_file) + "/annotation_" + camera_sensor + ".json"
    if os.path.exists(annotation_file) is False:
        raise ValueError(
            'Annotation file does not exist. Please annotate using the command rosrun atom_evaluation annotate.py -test_json {path_to_folder} -cs {souce_sensor} -si)')
        exit(0)
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
    f = open(annotation_file, 'r')
    eval_data = json.load(f)   
    
    # Deleting collections where the pattern is not found by all sensors:
    collections_to_delete = []
    for collection_key, collection in test_dataset['collections'].items():
        for sensor_key, sensor in test_dataset['sensors'].items():
            if not collection['labels'][sensor_key]['detected'] and (
                    sensor_key == camera_sensor or sensor_key == range_sensor):
                print(
                        Fore.RED + "Removing collection " + collection_key + ' -> pattern was not found in sensor ' +
                        sensor_key + ' (must be found in all sensors).' + Style.RESET_ALL)

                collections_to_delete.append(collection_key)
                break

    for collection_key in collections_to_delete:
        del test_dataset['collections'][collection_key]

    print(Fore.BLUE + '\nStarting evalutation...')
    print(Fore.WHITE)
    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print('{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}{:^25s}'.format('#', 'RMS', 'X Error', 'Y Error', 'X Standard Deviation',
                                                              'Y Standard Deviation'))
    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')

    # Declare output dict to save the evaluation data if desired
    output_dict = {}
    output_dict['ground_truth_pts'] = {}

    delta_total = []

    from_frame = test_dataset['calibration_config']['sensors'][camera_sensor]['link']
    to_frame = test_dataset['calibration_config']['sensors'][range_sensor]['link']
    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        vel2cam = getTransform(from_frame, to_frame,
                               test_dataset['collections'][collection_key]['transforms'])
        pts_in_image = rangeToImage(collection, test_json_file, range_sensor, camera_sensor, vel2cam)

        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        filename = os.path.dirname(test_json_file) + '/' + collection['data'][camera_sensor]['data_file']

        # print(filename)

        image = cv2.imread(filename)
        limits_on_image = eval_data['ground_truth_pts'][collection_key]
        # print(limits_on_image)

        # Clear image annotations
        image = cv2.imread(filename)

        output_dict['ground_truth_pts'][collection_key] = {}
        for i, pts in limits_on_image.items():
            pts = np.array(pts)
            if pts.size == 0:
                continue

            x = pts[:, 0]
            y = pts[:, 1]
            coefficients = np.polyfit(x, y, 3)
            poly = np.poly1d(coefficients)
            new_x = np.linspace(np.min(x), np.max(x), 5000)
            new_y = poly(new_x)

            if show_images:
                for idx in range(0, len(new_x)):
                    image = cv2.circle(image, (int(new_x[idx]), int(new_y[idx])), 3, (0, 0, 255), -1)

            output_dict['ground_truth_pts'][collection_key][i] = []
            for idx in range(0, len(new_x)):
                output_dict['ground_truth_pts'][collection_key][i].append([new_x[idx], new_y[idx]])

        # ---------------------------------------
        # --- Evaluation metrics - reprojection error
        # ---------------------------------------
        # -- For each reprojected limit point, find the closest ground truth point and compute the distance to it
        delta_pts = []
        for idx in range(0, pts_in_image.shape[1]):
            target_pt = pts_in_image[:, idx]
            target_pt = np.reshape(target_pt[0:2], (2, 1))
            min_dist = 1e6

            # Don't consider point that are re-projected outside of the image
            if target_pt[0] > image.shape[1] or target_pt[0] < 0 or target_pt[1] > image.shape[0] or target_pt[1] < 0:
                continue

            for i, pts in output_dict['ground_truth_pts'][collection_key].items():
                dist = np.min(distance.cdist(target_pt.transpose(), pts, 'euclidean'))

                if dist < min_dist:
                    min_dist = dist
                    arg = np.argmin(distance.cdist(target_pt.transpose(), pts, 'euclidean'))
                    closest_pt = pts[arg]

            diff = (closest_pt - target_pt.transpose())[0]

            delta_pts.append(diff)
            delta_total.append(diff)

            if show_images is True:
                image = cv2.line(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])),
                                 (int(closest_pt[0]), int(closest_pt[1])), (0, 255, 255), 3)

        if len(delta_pts) == 0:
            print('No LiDAR point mapped into the image for collection ' + str(collection_key))
            continue

        # ---------------------------------------
        # --- Compute error metrics
        # ---------------------------------------
        total_pts = len(delta_pts)
        delta_pts = np.array(delta_pts, np.float32)
        avg_error_x = np.sum(np.abs(delta_pts[:, 0])) / total_pts
        avg_error_y = np.sum(np.abs(delta_pts[:, 1])) / total_pts
        stdev = np.std(delta_pts, axis=0)

        # Print error metrics
        print(
            '{:^25s}{:^25s}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format(collection_key, '-', avg_error_x, avg_error_y,
                                                                        stdev[0], stdev[1]))

        # ---------------------------------------
        # --- Drawing ...
        # ---------------------------------------
        if show_images is True:
            for idx in range(0, pts_in_image.shape[1]):
                image = cv2.circle(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])), 5, (255, 0, 0), -1)

            window_name = "Lidar to Camera reprojection - collection " + str(collection_key)
            cv2.imshow(window_name, image)
            cv2.waitKey()
            cv2.destroyWindow(winname=window_name)

    total_pts = len(delta_total)
    delta_total = np.array(delta_total, float)
    avg_error_x = np.sum(np.abs(delta_total[:, 0])) / total_pts
    avg_error_y = np.sum(np.abs(delta_total[:, 1])) / total_pts
    stdev = np.std(delta_total, axis=0)
    rms = np.sqrt((delta_total ** 2).mean())

    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print('{:^25}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format(
        'All', rms, avg_error_x, avg_error_y, stdev[0], stdev[1]))
    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print('Ending script...')
    sys.exit()
    # print("Press ESC to quit and close all open windows.")

    # while True:
        # k = cv2.waitKey(0) & 0xFF
        # if k == 27:
            # cv2.destroyAllWindows()
            # break
    # Save evaluation data
    # if use_annotation is True:
    #     createJSONFile(eval_file, output_dict)
