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
import math
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

def draw_registration_result(source, target, transformation, initial_transformation):
    """
    Visualization tool for open3d
    """
    source_temp = copy.deepcopy(source)
    source_initial_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([0, 1, 1])
    source_initial_temp.paint_uniform_color([0, 0.5, 0.5])
    target_temp.paint_uniform_color([1, 0, 0])
    source_temp.transform(transformation)
    source_initial_temp.transform(initial_transformation)
    o3d.visualization.draw_geometries([source_initial_temp, source_temp, target_temp],
                                        zoom=0.4459,
                                        front=[0.9288, -0.2951, -0.2242],
                                        lookat=[1.6784, 2.0612, 1.4451],
                                        up=[-0.3402, -0.9189, -0.1996])

def pick_points(pcd):
    """
    Open a window to pick points in order to align two pointclouds
    """
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press q for close the window")
    print("3) If you desire to not use this collection, please don't choose any points in both pointclouds")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def ICP_calibration(source_point_cloud, target_point_cloud, threshold, T_init, show_images):
    """
    Use ICP to estimate the transformation between two range sensors
    """
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_point_cloud, target_point_cloud, threshold, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)

    if show_images:
        draw_registration_result(source_point_cloud, target_point_cloud, reg_p2p.transformation, T_init)

    return reg_p2p


def save_ICP_calibration(dataset, sensor_source, sensor_target, transform, json_file, descriptor):
    """
    Save a JSON file with the data of the ICP calibration
    """
    res = atomicTfFromCalibration(dataset, sensor_target, sensor_source, transform)
    child_link = dataset['calibration_config']['sensors'][sensor_source]['child_link']
    parent_link = dataset['calibration_config']['sensors'][sensor_source]['parent_link']
    frame = parent_link + '-' + child_link
    quat = tf.transformations.quaternion_from_matrix(res)
    for collection_key, collection in dataset['collections'].items():
        dataset['collections'][collection_key]['transforms'][frame]['quat'] = quat
        dataset['collections'][collection_key]['transforms'][frame]['trans'] = res[0:3, 3]

    # Save results to a json file
    filename_results_json = os.path.dirname(json_file) + '/ICP_calibration_' + descriptor + '.json'
    saveResultsJSON(filename_results_json, dataset)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-ss", "--sensor_source", help="Name of the sensor to be aligned.", type=str, required=True)
    ap.add_argument("-st", "--sensor_target", help="Name of the anchored sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    show_images = args['show_images']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Define variables
    threshold = 0.1
    initial_transforms = np.zeros((4, 4), np.float32)
    aligned_transforms = np.zeros((4, 4), np.float32)
    initial_min_rmse = math.inf
    initial_min_transform = None
    aligned_min_rmse = math.inf
    aligned_min_transform = None
    source_frame = dataset['calibration_config']['sensors'][args['sensor_source']]['link']
    target_frame = dataset['calibration_config']['sensors'][args['sensor_target']]['link']
    collections_list = list(dataset['collections'].keys())
    used_datasets = 0

    for idx, selected_collection_key in enumerate(collections_list):
        print('\nCalibrating collection ' + str(selected_collection_key))
        # Get the transformation from target to frame
        T_target_to_source_initial = getTransform(target_frame, source_frame,
                                        dataset['collections'][selected_collection_key]['transforms'])

        # Retrieve file names
        ss_filename = os.path.dirname(
            args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_source']]['data_file']
        st_filename = os.path.dirname(
            args['json_file']) + '/' + dataset['collections'][selected_collection_key]['data'][args['sensor_target']]['data_file']

        print('Reading point cloud from ' + ss_filename)
        print('Reading point cloud from ' + st_filename)

        source_point_cloud = o3d.io.read_point_cloud(ss_filename)
        target_point_cloud = o3d.io.read_point_cloud(st_filename)

        # Calibrate using the dataset initial estimate
        print('Using ICP to calibrate using the initial transform defined in the dataset')
        print('T_target_to_source_initial = \n' + str(T_target_to_source_initial))
        reg_p2p_initial = ICP_calibration(source_point_cloud, target_point_cloud, threshold, T_target_to_source_initial, show_images)

        # Append transforms
        # initial_transforms.append(reg_p2p_initial.transformation)
        initial_transforms += reg_p2p_initial.transformation
        if reg_p2p_initial.inlier_rmse < initial_min_rmse:
            initial_min_transform = reg_p2p_initial.transformation
            initial_min_rmse = reg_p2p_initial.inlier_rmse

        # Align the pointclouds
        print('\nAligning pointclouds for a better calibration')
        source_picked_points = pick_points(source_point_cloud)
        target_picked_points = pick_points(target_point_cloud)

        # Conditions of alignment
        if len(source_picked_points) == 0 and len(target_picked_points) == 0:
            print('\nYou have not chosen any points, so this collection will be ignored')
            continue
        elif not (len(source_picked_points) >= 3 and len(target_picked_points) >= 3):
            print('\nYou have chosen less than 3 points in at least one of the last two pointclouds, please redo them.')
            collections_list.insert(idx, selected_collection_key)
            continue            
        if not (len(source_picked_points) == len(target_picked_points)):
            print(f'\nYou have chosen {len(source_picked_points)} and {len(target_picked_points)} points, which needed to be equal, please redo them.')
            collections_list.insert(idx, selected_collection_key)
            continue           

        corr = np.zeros((len(source_picked_points), 2))
        corr[:, 0] = source_picked_points
        corr[:, 1] = target_picked_points

        # Estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        T_target_to_source_aligned = p2p.compute_transformation(source_point_cloud, target_point_cloud,
                                                o3d.utility.Vector2iVector(corr))
            
        # Calibrate using the new transform
        print('Using ICP to calibrate using the transform calculated by the correspondence of points done by the user')
        print('T_target_to_source_aligned = \n' + str(T_target_to_source_aligned))
        reg_p2p_aligned = ICP_calibration(source_point_cloud, target_point_cloud, threshold, T_target_to_source_aligned, show_images)

        # Append transforms
        aligned_transforms += reg_p2p_aligned.transformation
        if reg_p2p_aligned.inlier_rmse < aligned_min_rmse:
            aligned_min_transform = reg_p2p_aligned.transformation
            aligned_min_rmse = reg_p2p_aligned.inlier_rmse
        
        # Define dataset as used
        used_datasets += 1

    # Average the initial transforms
    initial_transform = initial_transforms / used_datasets 
    aligned_transform = aligned_transforms / used_datasets 

    # Saving Json files
    save_ICP_calibration(dataset, args['sensor_source'], args['sensor_target'], initial_transform, json_file, 'initial_average')
    save_ICP_calibration(dataset, args['sensor_source'], args['sensor_target'], initial_min_transform, json_file, 'initial_best')
    save_ICP_calibration(dataset, args['sensor_source'], args['sensor_target'], aligned_transform, json_file, 'aligned_average')
    save_ICP_calibration(dataset, args['sensor_source'], args['sensor_target'], aligned_min_transform, json_file, 'aligned_best')



if __name__ == '__main__':
   main() 