#!/usr/bin/env python3

"""
Implementation of an ATOM-compatible alternative calibration method described by Li et. al originally written in MATLAB. Original repo: https://github.com/ihtishamaliktk/RWHE-Calib

This method solves the Robot-World/Hand-Eye calibration problem, with the formulation: AX = ZB, where:

A is the transformation from the gripper/flange/end-effector to the base;
B is the transformation from the camera to the pattern/target (in the paper, this is called "world". However, to be coherent with ATOM, we call it "pattern");

X is the transformation from the base of the robot to the pattern;
Z is the transformation from the gripper/flange/end-effector to the camera

"""

import argparse
import numpy as np

from atom_core.dataset_io import loadResultsJSON
from atom_core.atom import getTransform


def getCameraIntrinsicsFromDataset(dataset, camera):
    # Camera intrinsics (from the dataset) needed to calculate B
    K = np.zeros((3, 3), np.float32)
    D = np.zeros((5, 1), np.float32)
    K[0, :] = dataset['sensors'][camera]['camera_info']['K'][0:3]
    K[1, :] = dataset['sensors'][camera]['camera_info']['K'][3:6]
    K[2, :] = dataset['sensors'][camera]['camera_info']['K'][6:9]
    D[:, 0] = dataset['sensors'][camera]['camera_info']['D'][0:5]

    height = dataset['sensors'][camera]['camera_info']['height']
    width = dataset['sensors'][camera]['camera_info']['width']
    image_size = (height, width)

    return K, D, image_size

def main():
    
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", type=str, required=True,
                    help="Json file containing input dataset.")
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=str,
                    help="A string to be evaluated into a lambda function that receives a collection name as input and "
                    "returns True or False to indicate if the collection should be loaded (and used in the "
                    "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
                    "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    ap.add_argument("-bln", "--base_link_name", type=str, required=False, default="base_link", help="Name of the robot base link frame.")
    ap.add_argument("-eeln", "--end_effector_link_name", type=str, required=False, default="flange", help="Name of the end-effector link frame.")
    ap.add_argument("-c", "--camera", help="Camera sensor name.", type=str, required=True)
    
    args = vars(ap.parse_args())

    json_file = args['json_file']
    collection_selection_function = args['collection_selection_function']
    base_link_name = args['base_link_name']
    end_effector_link_name = args['end_effector_link_name']
    camera = args['camera']

    # Read dataset file
    dataset, json_file = loadResultsJSON(json_file, collection_selection_function)


    ########################################
    # GET A and B matrices to solve AX=ZB #
    ########################################

    # Get camera intrinsics from dataset
    K, D, image_size = getCameraIntrinsicsFromDataset(
        dataset=dataset,
        camera=camera
        )

    # Initialize list of A and B matrices (one for each collection)
    AA = [] 
    BB = []
    
    
    for collection_key, collection in dataset['collections'].items():
        
        print("Calculating A and B matrices for collection " + collection_key + "...")
        
        # A is the transform from the gripper to the robot's base. We can get it from the dataset
        transform_pool = collection['transforms']
        A = getTransform(
            from_frame=end_effector_link_name,
            to_frame=base_link_name,
            transforms=transform_pool
        )

        AA.append(A)

        # B is the transform from the camera to the calibration pattern. We can get it from the solvePnP() method, using the detections in the dataset


if __name__ == '__main__':
    main()