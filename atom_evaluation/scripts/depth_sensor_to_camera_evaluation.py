#!/usr/bin/env python3

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import json
import os
import argparse
from collections import OrderedDict

import numpy as np
import atom_core.atom
import cv2
from rospy_message_converter import message_converter
from atom_core.vision import projectToCamera
from scipy.spatial import distance
from colorama import Style, Fore
from image_geometry import PinholeCameraModel
from atom_core.naming import generateKey
from atom_calibration.collect.label_messages import convertDepthImage16UC1to32FC1


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------
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
    ap.add_argument("-ds", "--depth_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-cs", "--rgb_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)
    # ap.add_argument("-ua", "--use_annotation", help="If true, the limit points will be manually annotated.",
    #                 action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    depth_sensor = args['depth_sensor']
    rgb_sensor = args['rgb_sensor']
    show_images = args['show_images']
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

    annotation_file = os.path.dirname(test_json_file) + "/annotation_" + rgb_sensor + ".json"
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
        # Loads a json file containing the evaluation data
    f = open(annotation_file, 'r')
    eval_data = json.load(f)

    pinhole_camera_model = PinholeCameraModel()
    pinhole_camera_model.fromCameraInfo(
        message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                            train_dataset['sensors'][depth_sensor]['camera_info']))

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

    from_frame = test_dataset['calibration_config']['sensors'][rgb_sensor]['link']
    to_frame = test_dataset['calibration_config']['sensors'][depth_sensor]['link']
    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        depth2cam = atom_core.atom.getTransform(from_frame, to_frame,
                                                test_dataset['collections'][collection_key]['transforms'])
        pts_in_image = depthToImage(collection, test_json_file, depth_sensor,
                                    rgb_sensor, depth2cam, pinhole_camera_model)

        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        filename = os.path.dirname(test_json_file) + '/' + collection['data'][rgb_sensor]['data_file']
        print(filename)
        image = cv2.imread(filename)
        limits_on_image = eval_data['ground_truth_pts'][collection_key]

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
            print('No Depth point mapped into the image for collection ' + str(collection_key))
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

            cv2.imshow("Depth to Camera reprojection - collection " + str(collection_key), image)
            cv2.waitKey()

    total_pts = len(delta_total)
    delta_total = np.array(delta_total, float)
    avg_error_x = np.sum(np.abs(delta_to[:, 0])) / total_pts
    avg_error_y = np.sum(np.abs(delta_total[:, 1])) / total_pts
    stdev = np.std(delta_total, axis=0)
    rms = np.sqrt((delta_total ** 2).mean())

    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')
    print('{:^25}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}{:^25.4f}'.format(
        'All', rms, avg_error_x, avg_error_y, stdev[0], stdev[1]))
    print(
        '------------------------------------------------------------------------------------------------------------------------------------------------------------')

    print("Press ESC to quit and close all open windows.")

    while True:
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            cv2.destroyAllWindows()
            break
