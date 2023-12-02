#!/usr/bin/env python3

"""
Stereo calibration with Kalibr
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import numpy as np
import argparse
import json
import os
import tf
import yaml
import sys

# from colorama import Style, Fore
# from collections import OrderedDict
from atom_evaluation.utilities import atomicTfFromCalibration
# from atom_core.atom import getTransform
from atom_core.dataset_io import saveAtomDataset


def KalibrStereoCalibration(camchain_file):

    with open(camchain_file, 'r') as f:
        try:
            kalibr = yaml.load(f, Loader=yaml.SafeLoader)
        except yaml.YAMLError as exc:
            print(exc)
            sys.exit(2)

    # ---------------------------------------
    # --- Get intrinsic data for both sensors
    # ---------------------------------------

    K = kalibr['cam0']['intrinsics']
    D = kalibr['cam0']['distortion_coeffs']

    K_l = np.zeros((3, 3), np.float32)
    D_l = np.zeros((5, 1), np.float32)

    K_l[0, 0] = K[0]
    K_l[0, 2] = K[2]
    K_l[1, 1] = K[1]
    K_l[1, 2] = K[3]
    K_l[2, 2] = 1

    D_l[:4, 0] = D

    # Target sensor
    K = kalibr['cam1']['intrinsics']
    D = kalibr['cam1']['distortion_coeffs']

    K_r = np.zeros((3, 3), np.float32)
    D_r = np.zeros((5, 1), np.float32)

    K_r[0, 0] = K[0]
    K_r[0, 2] = K[2]
    K_r[1, 1] = K[1]
    K_r[1, 2] = K[3]
    K_r[2, 2] = 1

    D_r[:4, 0] = D

    # Transformation
    T = np.array(kalibr['cam1']['T_cn_cnm1'])

    camera_model = dict([('K_l', K_l), ('K_r', K_r), ('D_l', D_l), ('D_r', D_r), ('T', T)])
    left_camera = kalibr['cam0']['rostopic'].split('/')[1]
    right_camera = kalibr['cam1']['rostopic'].split('/')[1]

    return camera_model, left_camera, right_camera


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str, required=True)
    ap.add_argument("-k", "--kalibr", help="Kalibr calibration yaml file.", type=str, required=True)

    # ap.add_argument("-rc", "--right_camera", help="Right camera sensor name.", type=str, required=True)
    # ap.add_argument("-lc", "--left_camera", help="Left camera sensor name.", type=str, required=True)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    kalibr_file = args['kalibr']

    # Read json file
    with open(json_file, 'r') as f:
        dataset = json.load(f)

    calib_model, left_camera, right_camera = KalibrStereoCalibration(kalibr_file)

    K_r = calib_model['K_r']
    D_r = calib_model['D_r']
    K_l = calib_model['K_l']
    D_l = calib_model['D_l']
    T = calib_model['T']

    res = atomicTfFromCalibration(dataset, right_camera, left_camera, T)

    # Re-write atomic transformation to the json file ...
    dataset['sensors'][right_camera]['camera_info']['K'][0:3] = K_r[0, :]
    dataset['sensors'][right_camera]['camera_info']['K'][3:6] = K_r[1, :]
    dataset['sensors'][right_camera]['camera_info']['K'][6:9] = K_r[2, :]

    dataset['sensors'][left_camera]['camera_info']['K'][0:3] = K_l[0, :]
    dataset['sensors'][left_camera]['camera_info']['K'][3:6] = K_l[1, :]
    dataset['sensors'][left_camera]['camera_info']['K'][6:9] = K_l[2, :]

    dataset['sensors'][right_camera]['camera_info']['D'][0:5] = D_r[:, 0]
    dataset['sensors'][left_camera]['camera_info']['D'][0:5] = D_l[:, 0]

    child_link = dataset['calibration_config']['sensors'][left_camera]['child_link']
    parent_link = dataset['calibration_config']['sensors'][left_camera]['parent_link']
    frame = parent_link + '-' + child_link
    quat = tf.transformations.quaternion_from_matrix(res)
    for collection_key, collection in dataset['collections'].items():
        dataset['collections'][collection_key]['transforms'][frame]['quat'] = quat
        dataset['collections'][collection_key]['transforms'][frame]['trans'] = res[0:3, 3]

        # dataset['collections'][collection_key]['transforms'][frame]['quat'] = [ 0.00082083, -0.00728841, -0.00023114,  0.99997308]
        # dataset['collections'][collection_key]['transforms'][frame]['trans'] =[0 , 0, -0.00028282]

    # Save results to a json file
    filename_results_json = os.path.dirname(json_file) + '/kalibr_calibration.json'
    saveAtomDataset(filename_results_json, dataset)
