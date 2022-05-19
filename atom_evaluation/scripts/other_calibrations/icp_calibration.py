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

def main():
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

    print(source_frame)
    print('T_lidar1_to_lidar2 = ' + str(T_lidar1_to_lidar2))

    # TODO use ICP for all collections and take average
    ss_filename = os.path.dirname(
        args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_source']]['data_file']
    st_filename = os.path.dirname(
        args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_target']]['data_file']

    print('Reading point cloud from ' + ss_filename)
    print('Reading point cloud from ' + st_filename)

    source_point_cloud = o3d.io.read_point_cloud(ss_filename)
    target_point_cloud = o3d.io.read_point_cloud(st_filename)
    threshold = 0.02

    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        target_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])

    draw_registration_result(source_point_cloud, target_point_cloud, T_lidar1_to_lidar2)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(source_point_cloud, target_point_cloud, threshold, T_lidar1_to_lidar2,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source_point_cloud, target_point_cloud, reg_p2p.transformation)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_point_cloud, target_point_cloud, threshold, T_lidar1_to_lidar2,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source_point_cloud, target_point_cloud, reg_p2p.transformation)

    # Save results to a json file
    # filename_results_json = os.path.dirname(json_file) + '/cv_calibration.json'
    # saveResultsJSON(filename_results_json, dataset)




if __name__ == '__main__':
   main() 