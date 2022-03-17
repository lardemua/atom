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
import ros_numpy

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


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def depthInImage(collection, json_file, ss, pinhole_camera_model):
    filename = os.path.dirname(json_file) + '/' + collection['data'][ss]['data_file']

    cv_image_int16_tenths_of_millimeters = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    img = convertDepthImage16UC1to32FC1(cv_image_int16_tenths_of_millimeters,
                                        scale=10000.0)

    idxs = collection['labels'][ss]['idxs_limit_points']
    w = pinhole_camera_model.fullResolution()[0]
    # w = size[0]

    # initialize lists
    xs = []
    ys = []
    for idx in idxs:  # iterate all points
        # convert from linear idx to x_pix and y_pix indices.
        y_pix = int(idx / w)
        x_pix = int(idx - y_pix * w)
        xs.append(x_pix)
        ys.append(y_pix)

    points_in_depth = np.array((xs, ys), dtype=float)
    # print(points_in_depth)
    return points_in_depth

def convert_from_uvd(cx, cy, fx, fy, xpix, ypix, d):
    # From http://www.open3d.org/docs/0.7.0/python_api/open3d.geometry.create_point_cloud_from_depth_image.html

    x_over_z = (xpix - cx) / fx
    y_over_z = (ypix - cy) / fy
    z = d
    x = x_over_z * z
    y = y_over_z * z
    return x, y, z

def depthToImage(collection, json_file, ss, ts, tf, pinhole_camera_model):
    filename = os.path.dirname(json_file) + '/' + collection['data'][ss]['data_file']

    cv_image_int16_tenths_of_millimeters = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    img = convertDepthImage16UC1to32FC1(cv_image_int16_tenths_of_millimeters,
                                                            scale=10000.0)

    idxs = collection['labels'][ss]['idxs_limit_points']

    f_x = pinhole_camera_model.fx()
    f_y = pinhole_camera_model.fy()
    c_x = pinhole_camera_model.cx()
    c_y = pinhole_camera_model.cy()
    w = pinhole_camera_model.fullResolution()[0]
    # w = size[0]

    # initialize lists
    xs = []
    ys = []
    zs = []
    for idx in idxs:  # iterate all points

        # convert from linear idx to x_pix and y_pix indices.
        y_pix = int(idx / w)
        x_pix = int(idx - y_pix * w)

        # get distance value for this pixel coordinate
        distance = img[y_pix, x_pix]

        # compute 3D point and add to list of coordinates xs, ys, zs
        x, y, z = convert_from_uvd(c_x, c_y, f_x, f_y, x_pix, y_pix, distance)
        xs.append(x)
        ys.append(y)
        zs.append(z)

    homogeneous = np.ones((len(xs)))
    points_in_depth = np.array((xs, ys, zs, homogeneous), dtype=float)

    points_in_cam = np.dot(tf, points_in_depth)

    # -- Project them to the image
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    D = np.ndarray((5, 1), buffer=np.array(test_dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)

    pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

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
    ap.add_argument("-ds2", "--depth_sensor_2", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ds1", "--depth_sensor_1", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    depth_sensor_2 = args['depth_sensor_2']
    depth_sensor_1 = args['depth_sensor_1']
    show_images = args['show_images']

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

    pinhole_camera_model_1 = PinholeCameraModel()
    pinhole_camera_model_1.fromCameraInfo(
        message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                            train_dataset['sensors'][depth_sensor_1]['camera_info']))

    pinhole_camera_model_2 = PinholeCameraModel()
    pinhole_camera_model_2.fromCameraInfo(
        message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                            train_dataset['sensors'][depth_sensor_2]['camera_info']))

    from_frame = test_dataset['calibration_config']['sensors'][depth_sensor_1]['link']
    to_frame = test_dataset['calibration_config']['sensors'][depth_sensor_2]['link']
    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))

    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        depth1_2_depth2 = atom_core.atom.getTransform(from_frame, to_frame,
                                              test_dataset['collections'][collection_key]['transforms'])

        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        filename = os.path.dirname(test_json_file) + '/' + collection['data'][depth_sensor_1]['data_file']
        print(filename)
        image = cv2.imread(filename)
        depth_pts_in_depth_img_1 = depthInImage(collection, test_json_file, depth_sensor_1, pinhole_camera_model_1)
        depth_pts_in_depth_img_2 = depthToImage(collection, test_json_file, depth_sensor_2, depth_sensor_1, depth1_2_depth2, pinhole_camera_model_2)

        # Clear image annotations
        image = cv2.imread(filename)

        delta_pts = []
        distances = []
        # lidar_points_xy = np.array([[pt['x'] for pt in lidar_points_in_pattern], [pt['y'] for pt in lidar_points_in_pattern]],
        #                                                 np.float)
        for idx in range(depth_pts_in_depth_img_2.shape[1]):
            lidar_pt = np.reshape(depth_pts_in_depth_img_2[0:2, idx], (1, 2))
            delta_pts.append(np.min(distance.cdist(lidar_pt, depth_pts_in_depth_img_1.transpose()[:, :2], 'euclidean')))
            coords = np.where(distance.cdist(lidar_pt, depth_pts_in_depth_img_1.transpose()[:, :2], 'euclidean') == np.min(
                distance.cdist(lidar_pt, depth_pts_in_depth_img_1.transpose()[:, :2], 'euclidean')))
            # if len(depth_pts_in_depth_img.transpose()[coords[1]])>1:
            #     min_dist_pt=depth_pts_in_depth_img.transpose()[coords[1],0]
            #     print(depth_pts_in_depth_img.transpose()[coords[1][0]])
            #     print(depth_pts_in_depth_img.transpose()[coords[1]],min_dist_pt)
            # else:
            min_dist_pt=depth_pts_in_depth_img_1.transpose()[coords[1]][0]
            # print(min_dist_pt)
            dist = abs(lidar_pt - min_dist_pt)
            distances.append(dist)
            delta_total.append(dist)

            if show_images:
                image = cv2.line(image, (int(lidar_pt.transpose()[0]), int(lidar_pt.transpose()[1])),
                                 (int(min_dist_pt[0]), int(min_dist_pt[1])), (0, 255, 255), 3)

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
        if show_images is True:
            for idx in range(0, depth_pts_in_depth_img_2.shape[1]):
                image = cv2.circle(image, (int(depth_pts_in_depth_img_2[0, idx]), int(depth_pts_in_depth_img_2[1, idx])), 5, (255, 0, 0), -1)
            for idx in range(0, depth_pts_in_depth_img_1.shape[1]):
                image = cv2.circle(image, (int(depth_pts_in_depth_img_1[0, idx]), int(depth_pts_in_depth_img_1[1, idx])), 5, (0, 0, 255),
                                   -1)
            cv2.imshow("Lidar to Camera reprojection - collection " + str(collection_key), image)
            cv2.waitKey()


    total_pts = len(delta_total)
    delta_total = np.array(delta_total, np.float32)
    avg_error = np.sum(np.abs(delta_total)) / total_pts
    rms = np.sqrt((delta_total ** 2).mean())

    delta_xy = np.array(delta_total, np.float32)
    delta_xy = delta_xy[:, 0]
    avg_error_x = np.sum(np.abs(delta_xy[:, 0])) / total_pts
    avg_error_y = np.sum(np.abs(delta_xy[:, 1])) / total_pts
    stdev_xy = np.std(delta_xy, axis=0)

    print('-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print(
        '{:^25s}{:^25f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format('All', rms,
                                                                                      avg_error, avg_error_x,
                                                                                      avg_error_y,
                                                                                      stdev_xy[0], stdev_xy[1]))
    print(
        '-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print("Press ESC to quit and close all open windows.")

    while True:
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            cv2.destroyAllWindows()
            break
# Save evaluation data
# if use_annotation is True:
#     createJSONFile(eval_file, output_dict)
