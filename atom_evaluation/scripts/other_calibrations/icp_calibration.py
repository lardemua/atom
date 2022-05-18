#!/usr/bin/env python3

"""
Stereo calibration from opencv
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import argparse
import copy
import json
import os
from collections import OrderedDict

import numpy as np
import open3d as o3d
import cv2
import tf
from colorama import Style, Fore
from atom_evaluation.utilities import atomicTfFromCalibration
from atom_core.atom import getTransform
from atom_core.dataset_io import saveResultsJSON


def cvStereoCalibrate(objp):
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints_l = []  # 2d points in image plane.
    imgpoints_r = []  # 2d points in image plane.

    for collection_key, collection in dataset['collections'].items():
        # Find the chess board corners
        n_points = int(dataset['calibration_config']['calibration_pattern']['dimension']['x']) * \
            int(dataset['calibration_config']['calibration_pattern']['dimension']['y'])
        image_points_r = np.ones((n_points, 2), np.float32)
        image_points_l = np.ones((n_points, 2), np.float32)

        for idx, point in enumerate(collection['labels'][right_camera]['idxs']):
            image_points_r[idx, 0] = point['x']
            image_points_r[idx, 1] = point['y']

        for idx, point in enumerate(collection['labels'][left_camera]['idxs']):
            image_points_l[idx, 0] = point['x']
            image_points_l[idx, 1] = point['y']

        imgpoints_l.append(image_points_l)
        imgpoints_r.append(image_points_r)
        objpoints.append(objp)

    # ---------------------------------------
    # --- Get intrinsic data for both sensors
    # ---------------------------------------
    # Source sensor
    K_r = np.zeros((3, 3), np.float32)
    D_r = np.zeros((5, 1), np.float32)
    K_r[0, :] = dataset['sensors'][right_camera]['camera_info']['K'][0:3]
    K_r[1, :] = dataset['sensors'][right_camera]['camera_info']['K'][3:6]
    K_r[2, :] = dataset['sensors'][right_camera]['camera_info']['K'][6:9]
    D_r[:, 0] = dataset['sensors'][right_camera]['camera_info']['D'][0:5]

    # Target sensor
    K_l = np.zeros((3, 3), np.float32)
    D_l = np.zeros((5, 1), np.float32)
    K_l[0, :] = dataset['sensors'][left_camera]['camera_info']['K'][0:3]
    K_l[1, :] = dataset['sensors'][left_camera]['camera_info']['K'][3:6]
    K_l[2, :] = dataset['sensors'][left_camera]['camera_info']['K'][6:9]
    D_l[:, 0] = dataset['sensors'][left_camera]['camera_info']['D'][0:5]

    height = dataset['sensors'][right_camera]['camera_info']['height']
    width = dataset['sensors'][right_camera]['camera_info']['width']
    image_size = (height, width)

    print('\n---------------------\n Starting stereo calibration ...')

    # Extrinsic stereo calibration
    stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                            cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS)

    ret, K_l, D_l, K_r, D_r, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_l,
                                                              imgpoints_r, K_l, D_l, K_r,
                                                              D_r, image_size,
                                                              criteria=stereocalib_criteria, flags=flags)

    print('\n---------------------\n Done!\n\n------\nCalibration results:\n------\n')

    print('K_left', K_l)
    print('D_left', D_l)
    print('K_right', K_r)
    print('D_right', D_r)
    print('R', R)
    print('T', T)
    print('E', E)
    print('F', F)

    camera_model = dict([('K_l', K_l), ('K_r', K_r), ('D_l', D_l),
                         ('D_r', D_r), ('R', R), ('T', T),
                         ('E', E), ('F', F)])

    cv2.destroyAllWindows()
    return camera_model


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-ss", "--sensor_source", help="Right camera sensor name.", type=str, required=True)
    ap.add_argument("-st", "--sensor_target", help="Left camera sensor name.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # get the tranformation from lidar1 to lidar2
    selected_collection_key = list(dataset['collections'].keys())[0]
    source_frame = dataset['calibration_config']['sensors'][args['sensor_source']]['link']
    target_frame = dataset['calibration_config']['sensors'][args['sensor_target']]['link']
    T_lidar1_to_lidar2 = getTransform(source_frame, target_frame,
                                      dataset['collections'][selected_collection_key]['transforms'])

#     T_lidar1_to_lidar2 = getTransform(target, target_frame,
#                                       dataset['collections'][selected_collection_key]['transforms'])
#

    print('T_lidar1_to_lidar2 = ' + str(T_lidar1_to_lidar2))

    # TODO use ICP for all colelctions and take average
    ss_filename = os.path.dirname(
        args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_source']]['data_file']
    st_filename = os.path.dirname(
        args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_target']]['data_file']

    print('Reading point cloud from ' + ss_filename)
    print('Reading point cloud from ' + st_filename)

    source_point_cloud = o3d.io.read_point_cloud(ss_filename)
    target_point_cloud = o3d.io.read_point_cloud(st_filename)
    threshold = 0.02
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                            [-0.139, 0.967, -0.215, 0.7],
                            [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])

    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])

    draw_registration_result(source_point_cloud, target_point_cloud, T_lidar1_to_lidar2)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(source_point_cloud, target_point_cloud, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source_point_cloud, target_point_cloud, reg_p2p.transformation)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_point_cloud, target_point_cloud, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source_point_cloud, target_point_cloud, reg_p2p.transformation)

    # Save results to a json file
    # filename_results_json = os.path.dirname(json_file) + '/cv_calibration.json'
    # saveResultsJSON(filename_results_json, dataset)

