#!/usr/bin/env python3

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
import tf

from colorama import Style, Fore
from collections import OrderedDict
from atom_evaluation.utilities import atomicTfFromCalibration
from atom_core.atom import getTransform
from atom_core.dataset_io import saveAtomDataset
from atom_core.geometry import traslationRodriguesToTransform


# def cvHandEyeCalibrate(objp, dataset, camera, pattern, number_of_corners):
#     # Arrays to store object points and image points from all the images.
#     objpoints = []  # 3d point in real world space
#     imgpoints_camera = []  # 2d points in image plane.
#
#     for collection_key, collection in dataset['collections'].items():
#
#         tmp_imgpoints_camera = np.ones((number_of_corners, 2), np.float32)  # temporary var
#
#         for idx, point in enumerate(collection['labels'][pattern][camera]['idxs']):
#             tmp_imgpoints_camera[idx, 0] = point['x']
#             tmp_imgpoints_camera[idx, 1] = point['y']
#
#         imgpoints_camera.append(tmp_imgpoints_camera)
#         objpoints.append(objp)  # I don't understand this
#
#     # print(imgpoints_camera)
#
#     # ---------------------------------------
#     # --- Get intrinsic data for the sensor
#     # ---------------------------------------
#     # Source sensor
#     K = np.zeros((3, 3), np.float32)
#     D = np.zeros((5, 1), np.float32)
#     K[0, :] = dataset['sensors'][camera]['camera_info']['K'][0:3]
#     K[1, :] = dataset['sensors'][camera]['camera_info']['K'][3:6]
#     K[2, :] = dataset['sensors'][camera]['camera_info']['K'][6:9]
#     D[:, 0] = dataset['sensors'][camera]['camera_info']['D'][0:5]
#
#     height = dataset['sensors'][camera]['camera_info']['height']
#     width = dataset['sensors'][camera]['camera_info']['width']
#     image_size = (height, width)
#
#     #############################
#     # Undistort detection points
#     #############################
#
#     print(f'Calculating undistorted corner detection coordinates...')
#     undistorted_imgpoints_camera = []  # Init matrix with points for every collection
#
#     for i in range(0, len(imgpoints_camera)):  # Iterate through the collections
#         tmp_undistorted_imgpoints_camera = cv2.undistortPoints(imgpoints_camera[i], K, D)
#         undistorted_imgpoints_camera.append(tmp_undistorted_imgpoints_camera)
#
#     #####################################
#     # Get transform from target to cam
#     #####################################
#
#     # NOTE: The documentation calls this the transform from world to cam because in their example (which is eye-in-hand), their pattern's frame is their world frame. This is not the case in ours. So when the documentation mentions the world to cam tf, it corresponds to our pattern/target to camera tf. (I think.)
#
#     # Use solvePnP() to get this transformation
#     # Need to check if these calculations (from here to l. 108) are correct
#
#     print('Calculating transform from camera to pattern for collection 006....')
#
#     _, rvec, tvec = cv2.solvePnP(objp, undistorted_imgpoints_camera[0], K, D)
#
#     print(f'rvec = {rvec}')
#
#     print(f'tvec = {tvec}')
#
#     # Convert it into an homogenous transform
#     cam_optical_frame_T_pattern = traslationRodriguesToTransform(tvec, rvec)
#
#     print(f'Transform from the camera\'s optical frame to the pattern frame = {cam_optical_frame_T_pattern}')
#
#     # Get tf from camera's optical frame to camera frame
#     tmp_transforms = dataset['collections']['006']['transforms']
#     cam_T_cam_optical_frame = getTransform('rgb_world_link', 'rgb_world_optical_frame', tmp_transforms)
#
#     print(f'Transform from cam to cam optical = {cam_T_cam_optical_frame}')
#
#     cam_T_pattern = np.dot(cam_T_cam_optical_frame, cam_optical_frame_T_pattern)
#
#     print(f'Transform from camera to pattern = {cam_T_pattern}')
#
#     # I think I need to invert the transform matrix
#     pattern_T_cam = np.linalg.inv(cam_T_pattern)
#
#     # Split into R and t matrices for the calibration function
#     pattern_R_cam = pattern_T_cam[0:3, 0:3]
#     pattern_t_cam = (pattern_T_cam[0:3, 3]).T
#
#     ################################################
#     # Get transform from base to gripper
#     ################################################
#
#     # Hard coded for now, testin w/ one collection before iterating through all collections
#     base_T_gripper = getTransform('base_link', 'flange', tmp_transforms)
#
#     # Split into R and t matrices for the calibration function
#     base_R_gripper = base_T_gripper[0:3, 0:3]
#     base_t_gripper = (base_T_gripper[0:3, 3]).T
#
#     ################################################
#     # Calibrate
#     ################################################
#
#     print('----------------\n\n\n\n')
#     print(pattern_R_cam)
#     print(pattern_R_cam.shape)
#
#     # Running with lists of np.arrays
#
#     # cv2.calibrateRobotWorldHandEye(pattern_R_cam, pattern_t_cam, base_R_gripper,
#     #                                base_t_gripper)  # THIS IS PRODUCING AN ERROR, SEE ISSUE #912
#
#     exit(0)


def cvHandEyeCalibrate(objp, dataset, camera, pattern, number_of_corners):

    # ---------------------------------------
    # --- Get intrinsic data for the sensor
    # ---------------------------------------
    # Source sensor
    K = np.zeros((3, 3), np.float32)
    D = np.zeros((5, 1), np.float32)
    K[0, :] = dataset['sensors'][camera]['camera_info']['K'][0:3]
    K[1, :] = dataset['sensors'][camera]['camera_info']['K'][3:6]
    K[2, :] = dataset['sensors'][camera]['camera_info']['K'][6:9]
    D[:, 0] = dataset['sensors'][camera]['camera_info']['D'][0:5]

    height = dataset['sensors'][camera]['camera_info']['height']
    width = dataset['sensors'][camera]['camera_info']['width']
    image_size = (height, width)

    # iterate all collections and create four lists of rotations and translations:
    list_cam_optical_frame_T_pattern_R = []
    list_cam_optical_frame_T_pattern_t = []
    list_base_T_gripper_R = []
    list_base_T_gripper_t = []

    for collection_key, collection in dataset['collections'].items():

        # ---------------------------------------
        # Get transform from target to cam
        # ---------------------------------------

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints_camera = []  # 2d points in image plane.

        tmp_imgpoints_camera = np.ones((number_of_corners, 2), np.float32)  # temporary var

        for idx, point in enumerate(collection['labels'][pattern][camera]['idxs']):
            tmp_imgpoints_camera[idx, 0] = point['x']
            tmp_imgpoints_camera[idx, 1] = point['y']

        imgpoints_camera.append(tmp_imgpoints_camera)
        objpoints.append(objp)  # Use the points defined in local pattern ref frame

        # print(imgpoints_camera)

        # ---------------------------------------
        # Undistort detection points
        # ---------------------------------------

        print(f'Calculating undistorted corner detection coordinates...')
        undistorted_imgpoints_camera = []  # Init matrix with points for every collection

        for i in range(0, len(imgpoints_camera)):  # Iterate through the collections
            tmp_undistorted_imgpoints_camera = cv2.undistortPoints(imgpoints_camera[i], K, D)
            undistorted_imgpoints_camera.append(tmp_undistorted_imgpoints_camera)

        # ---------------------------------------
        # Get transform from target to cam
        # ---------------------------------------
        # NOTE: The documentation calls this the transform from world to cam because in their example (which is eye-in-hand), their pattern's frame is their world frame. This is not the case in ours. So when the documentation mentions the world to cam tf, it corresponds to our pattern/target to camera tf. (I think.)

        # Use solvePnP() to get this transformation
        # Need to check if these calculations (from here to l. 108) are correct

        print('Calculating transform from camera to pattern for collection ' + collection_key)

        _, rvec, tvec = cv2.solvePnP(objp, undistorted_imgpoints_camera[0], K, D)

        # print(f'rvec = {rvec}')
        # print(f'tvec = {tvec}')

        # Convert it into an homogenous transform
        cam_optical_frame_T_pattern = traslationRodriguesToTransform(tvec, rvec)

        # NOTE fstrings are forbidden in atom :-)
        # print(f'cam_optical_frame_T_pattern = {cam_optical_frame_T_pattern}')

        # NOTE: you do not need to compute the transform from camera to camera_optical_frame

        # Split into R and t matrices for the calibration function
        cam_optical_frame_T_pattern_R = cam_optical_frame_T_pattern[0:3, 0:3]
        cam_optical_frame_T_pattern_t = (cam_optical_frame_T_pattern[0:3, 3]).T

        list_cam_optical_frame_T_pattern_R.append(cam_optical_frame_T_pattern_R)
        list_cam_optical_frame_T_pattern_t.append(cam_optical_frame_T_pattern_t)

        # ---------------------------------------
        # Get transform from base to gripper
        # ---------------------------------------

        # Hard coded for now, testin w/ one collection before iterating through all collections
        # NOTE: cannot test with a single collection. We need at least three. Check
        # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga41b1a8dd70eae371eba707d101729c36

        print('Calculating transform from the base to the gripper for collection ' + collection_key)

        base_T_gripper = getTransform('base_link', 'flange', collection['transforms'])

        # Split into R and t matrices for the calibration function
        base_T_gripper_R = base_T_gripper[0:3, 0:3]
        base_T_gripper_t = (base_T_gripper[0:3, 3]).T

        list_base_T_gripper_R.append(base_T_gripper_R)
        list_base_T_gripper_t.append(base_T_gripper_t)

    # ---------------------------------------
    # Calibrate
    # ---------------------------------------

    # Running with lists of np.arrays
    o = cv2.calibrateRobotWorldHandEye( list_cam_optical_frame_T_pattern_R,
                                        list_cam_optical_frame_T_pattern_t,
                                        list_base_T_gripper_R, list_base_T_gripper_t)

    return o

def getPatternConfig(dataset, pattern):
    # Pattern configs
    nx = dataset['calibration_config']['calibration_patterns'][pattern]['dimension']['x']
    ny = dataset['calibration_config']['calibration_patterns'][pattern]['dimension']['y']
    square = dataset['calibration_config']['calibration_patterns'][pattern]['size']
    inner_square = dataset['calibration_config']['calibration_patterns'][pattern]['inner_size']
    objp = np.zeros((nx * ny, 3), np.float32)
    # set of coordinates (w.r.t. the pattern frame) of the corners
    objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    return nx, ny, square, inner_square, objp

def getWantedTransformsFromOpenCVHandEyeCalib(dataset, calib_tf_base2pattern, calib_tf_gripper2opticalframe, base_link_name, optical_frame_name, sensor_link_name):

    world_link = dataset['calibration_config']['world_link']

    for collection_key, collection in dataset['collections'].items():
        print("Getting wanted transforms for collection " + collection_key + "...") 

        tfs = collection['transforms'] # Dict of tfs from a given collection
        # Getting the world to base transform (bTw)
        
        print("Getting world to base transform...")
        tf_world2base = getTransform(world_link, base_link_name, tfs)

        tf_world2pattern = np.dot(calib_tf_base2pattern, tf_world2base) # First wanted calib

        # Get the second transform (gripper to cam)
        print("Getting gripper to cam transform...")

        tf_opticalframe2cam = getTransform("rgb_hand_optical_frame", "rgb_hand_link", tfs)
        # print(tf_opticalframe2cam) # DEBUG

        tf_gripper2cam = np.dot(tf_opticalframe2cam, calib_tf_gripper2opticalframe)
        # print(tf_gripper2cam)

        return tf_world2pattern, tf_gripper2cam

        

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-c", "--camera", help="Camera sensor name.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)
    ap.add_argument("-p", "--pattern", help="Pattern to be used for calibration.", type=str, required=True)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    camera = args['camera']
    show_images = args['show_images']
    pattern = args['pattern']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    #########################################
    # DATASET PREPROCESSING
    #########################################

    nx, ny, square, inner_square, objp = getPatternConfig(dataset=dataset, pattern=pattern)

    # Remove partial detections (OpenCV does not support them)
    collections_to_delete = []

    number_of_corners = int(nx) * int(ny)
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if sensor_key != camera:
                continue

            if sensor['msg_type'] == 'Image' and collection['labels'][pattern][sensor_key]['detected']:
                if not len(collection['labels'][pattern][sensor_key]['idxs']) == number_of_corners:
                    print(
                        Fore.RED + 'Partial detection removed:' + Style.RESET_ALL + ' label from collection ' +
                        collection_key + ', sensor ' + sensor_key)

                    collections_to_delete.append(collection_key)
                    break

    for collection_key in collections_to_delete:
        del dataset['collections'][collection_key]

    # remove collections which do not have a pattern detection
    collections_to_delete = []
    for collection_key, collection in dataset['collections'].items():
        if not collection['labels'][pattern][args['camera']]['detected']:
            print('Collection ' + collection_key + ' pattern not detected on camera. Removing...')
            collections_to_delete.append(collection_key)

    for collection_key in collections_to_delete:
        del dataset['collections'][collection_key]

    print('\nUsing ' + str(len(dataset['collections'])) + ' collections.')

    # Compute OpenCV stereo calibration
    # Now does not crash!!!
    R_base2pattern, t_base2pattern, R_gripper2cam, t_gripper2cam = cvHandEyeCalibrate(objp=objp, dataset=dataset, camera=camera, pattern=pattern, number_of_corners=number_of_corners)

    ########################################
    # Get calibrated homogenous tfs
    ########################################

    calib_tf_base2pattern = np.zeros((4,4))
    calib_tf_base2pattern[0:3, 0:3] = R_base2pattern
    calib_tf_base2pattern[0:3, 3] = t_base2pattern.T
    calib_tf_base2pattern[3, 3] = 1

    # print(calib_tf_base2pattern)
        
    calib_tf_gripper2opticalframe = np.zeros((4,4))
    calib_tf_gripper2opticalframe[0:3, 0:3] = R_gripper2cam
    calib_tf_gripper2opticalframe[0:3, 3] = t_gripper2cam.T    
    calib_tf_gripper2opticalframe[3, 3] = 1

    calib_tf_world2pattern, calib_tf_gripper2cam = getWantedTransformsFromOpenCVHandEyeCalib(
        dataset = dataset,
        calib_tf_base2pattern = calib_tf_base2pattern,
        calib_tf_gripper2opticalframe = calib_tf_gripper2opticalframe,
        base_link_name = "base_link",
        optical_frame_name = camera + "_optical_frame",
        sensor_link_name = camera + "_link")

    # print(calib_tf_gripper2cam)

    # res_gripper2cam = atomicTfFromCalibration(dataset, camera, pattern, calib_tf_gripper2cam)


if __name__ == '__main__':

    main()

    # R = calib_model['R']
    # t = calib_model['T']
    # K_r = calib_model['K_r']
    # D_r = calib_model['D_r']
    # K_l = calib_model['K_l']
    # D_l = calib_model['D_l']

    # # Extract homogeneous transformation between cameras
    # calib_tf = np.zeros((4, 4), np.float32)
    # calib_tf[0:3, 0:3] = R
    # calib_tf[0:3, 3] = t.ravel()
    # calib_tf[3, 0:3] = 0
    # calib_tf[3, 3] = 1

    # res = atomicTfFromCalibration(dataset, right_camera, left_camera, calib_tf)

    # # Re-write atomic transformation to the json file ...
    # dataset['sensors'][right_camera]['camera_info']['K'][0:3] = K_r[0, :]
    # dataset['sensors'][right_camera]['camera_info']['K'][3:6] = K_r[1, :]
    # dataset['sensors'][right_camera]['camera_info']['K'][6:9] = K_r[2, :]

    # dataset['sensors'][left_camera]['camera_info']['K'][0:3] = K_l[0, :]
    # dataset['sensors'][left_camera]['camera_info']['K'][3:6] = K_l[1, :]
    # dataset['sensors'][left_camera]['camera_info']['K'][6:9] = K_l[2, :]

    # dataset['sensors'][right_camera]['camera_info']['D'][0:5] = D_r[:, 0]
    # dataset['sensors'][left_camera]['camera_info']['D'][0:5] = D_l[:, 0]

    # child_link = dataset['calibration_config']['sensors'][left_camera]['child_link']
    # parent_link = dataset['calibration_config']['sensors'][left_camera]['parent_link']
    # frame = parent_link + '-' + child_link
    # quat = tf.transformations.quaternion_from_matrix(res)
    # for collection_key, collection in dataset['collections'].items():
    #     dataset['collections'][collection_key]['transforms'][frame]['quat'] = quat
    #     dataset['collections'][collection_key]['transforms'][frame]['trans'] = res[0:3, 3]

    # # Save results to a json file
    # filename_results_json = os.path.dirname(json_file) + '/cv_calibration.json'
    # saveAtomDataset(filename_results_json, dataset)
