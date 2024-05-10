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
import cv2

from atom_core.dataset_io import loadResultsJSON
from atom_core.atom import getTransform
from atom_core.geometry import traslationRodriguesToTransform

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

def li_calib(AA,BB):
    # From here on out, the code is a direct translation of the MATLAB code

    n = len(AA) # n is the number of collections
    
    # Transform the 4x4xn AA 3-dimensional matrix to a 4x(4xn) 2-dimensional matrix like in the original script
    AA_2d = np.zeros((4, 4*n))
    for i in range(n):
        AA_2d[:, 4*i:4*i+4] = AA[i]
    # Do the same for BB
    BB_2d = np.zeros((4, 4*n))
    for i in range(n):
        BB_2d[:, 4*i:4*i+4] = BB[i]
    

    A = np.zeros((12*n, 24))
    b = np.zeros((12*n, 1))
    for i in range(1,n+1):
        Ra = AA_2d[0:3,4*i-4:4*i-1]
        Rb = BB_2d[0:3,4*i-4:4*i-1]
        ta = AA_2d[0:3,4*i-1:4*i]
        tb = BB_2d[0:3,4*i-1:4*i]

        A[12*i-12:12*i-3,0:9] = np.kron(Ra, np.eye(3))
        A[12*i-12:12*i-3,9:18] = np.kron(-1*np.eye(3), Rb.T)
        A[12*i-3:12*i,9:18] = np.kron(np.eye(3), tb.T)
        A[12*i-3:12*i,18:21] = -1*Ra
        A[12*i-3:12*i,21:24] = np.eye(3)
        
        b[12*i-3:12*i] = ta

    # The equivalent of the \ operator in MATsingular value decomposition ofLAB is the numpy linalg.solve function
    x = np.linalg.lstsq(A,b, rcond=None)
    x = x[0] # x[0] is all we need, as it is the array returned by matlab's "\""
    
    # Get X
    X = x[0:9].reshape((3,3)).T
    [u,s,v] = np.linalg.svd(X)
    X = u @ v.T
    if (np.linalg.det(X) < 0):
        X = u @ np.diag([1,1,-1]) @ v.T
    X = np.append(X, x[21:24], axis=1)
    X = np.append(X, np.array([[0,0,0,1]]), axis=0)

    # Get Y
    Y = x[9:18].reshape((3,3)).T
    [u,s,v] = np.linalg.svd(Y)
    Y = u @ v.T
    if (np.linalg.det(Y) < 0):
        Y = u @ np.diag([1,1,-1]) @ v.T
    Y = np.append(Y, x[21:24], axis=1)
    Y = np.append(Y, np.array([[0,0,0,1]]), axis=0)

    return X,Y


def main():
    
    ########################################
    # ARGUMENT PARSER #
    ########################################

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", type=str, required=True,
                    help="Json file containing input dataset.")
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=str,
                    help="A string to be evaluated into a lambda function that receives a collection name as input and "
                    "returns True or False to indicate if the collection should be loaded (and used in the "
                    "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
                    "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    ap.add_argument("-bl", "--base_link", type=str, required=False, default="base_link", help="Name of the robot base link frame.")
    ap.add_argument("-hl", "--hand_link", type=str, required=False, default="flange", help="Name of the hand link frame.")
    ap.add_argument("-c", "--camera", help="Camera sensor name.", type=str, required=True)
    ap.add_argument("-p", "--pattern", help="Pattern to be used for calibration.", type=str, required=True)
    
    args = vars(ap.parse_args())

    json_file = args['json_file']
    collection_selection_function = args['collection_selection_function']
    base_link = args['base_link']
    hand_link = args['hand_link']
    camera = args['camera']
    pattern = args['pattern']

    # Read dataset file
    dataset, json_file = loadResultsJSON(json_file, collection_selection_function)


    ########################################
    # DATASET PREPROCESSING #
    ########################################

    # Get camera intrinsics from the dataset, needed to calculate B
    K, D, image_size = getCameraIntrinsicsFromDataset(
        dataset=dataset,
        camera=camera
        )

    # Get pattern configuration from the dataset, also needed to calulate B
    
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

    # Remove collections which do not have a pattern detection
    collections_to_delete = []
    for collection_key, collection in dataset['collections'].items():
        if not collection['labels'][pattern][args['camera']]['detected']:
            print('Collection ' + collection_key + ' pattern not detected on camera. Removing...')
            collections_to_delete.append(collection_key)

    for collection_key in collections_to_delete:
        del dataset['collections'][collection_key]

    print('\nUsing ' + str(len(dataset['collections'])) + ' collections.')


    ########################################
    # GET A and B matrices to solve AX=ZB #
    ########################################

    # Initialize list of A and B matrices (one for each collection)
    AA = [] 
    BB = []
    
    
    for collection_key, collection in dataset['collections'].items():
        
        print("Calculating A and B matrices for collection " + collection_key + "...")
        
        # A is the transform from the gripper to the robot's base. We can get it from the dataset
        transform_pool = collection['transforms']
        A = getTransform(
            from_frame=hand_link,
            to_frame=base_link,
            transforms=transform_pool
        )

        AA.append(A)

        # B is the transform from the camera to the calibration pattern. We can get it from the solvePnP() method, using the detections in the dataset

        imgpoints_camera = [] #initialize list of 2d points in the image plane

        tmp_imgpoints_camera = np.ones((number_of_corners, 2), np.float32) # temporary var
        
        #NOTE: Check labels id (this works because detections are complete)
        for idx, point in enumerate(collection['labels'][pattern][camera]['idxs']):
            tmp_imgpoints_camera[idx, 0] = point['x']
            tmp_imgpoints_camera[idx, 1] = point['y']

        imgpoints_camera.append(tmp_imgpoints_camera)

        retval, rvec, tvec = cv2.solvePnP(objp, imgpoints_camera[0], K, D)

        tf_pattern2opticalframe = traslationRodriguesToTransform(tvec, rvec)

        # B is the inverse of pattern2opticalframe

        B = np.linalg.inv(tf_pattern2opticalframe)
        
        BB.append(B)

    X,Y = li_calib(AA,BB)


if __name__ == '__main__':
    main()