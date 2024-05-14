#!/usr/bin/env python3

"""
Eye-to-hand counterpart to ali_eye_in_hand.py
"""

import argparse
from copy import deepcopy
from colorama import Fore
import numpy as np
import cv2
from prettytable import PrettyTable
import tf
import math

from atom_core.dataset_io import filterCollectionsFromDataset, loadResultsJSON
from atom_core.atom import getTransform, getChain
from atom_core.geometry import traslationRodriguesToTransform
from atom_core.naming import generateKey
from atom_core.transformations import compareTransforms
from atom_core.utilities import compareAtomTransforms

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
    X = x[0:9].reshape((3,3))
    [u,s,v] = np.linalg.svd(X)
    s = np.array([[s[0], 0, 0],
                  [0, s[1], 0],
                  [0, 0, s[2]]])
    v = v.T
    X = u @ v.T
    if (np.linalg.det(X) < 0):
        X = u @ np.diag([1,1,-1]) @ v.T
    X = np.append(X, x[18:21], axis=1)
    X = np.append(X, np.array([[0,0,0,1]]), axis=0)

    # Get Y
    Y = x[9:18].reshape((3,3))
    [u,s,v] = np.linalg.svd(Y)
    s = np.array([[s[0], 0, 0],
                  [0, s[1], 0],
                  [0, 0, s[2]]])
    v = v.T
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
    ap.add_argument("-uic", "--use_incomplete_collections", action="store_true", default=False, help="Remove any collection which does not have a detection for all sensors.", )
    ap.add_argument("-ctgt", "--compare_to_ground_truth", action="store_true", help="If the system being calibrated is simulated, directly compare the TFs to the ground truth.")
    
    args = vars(ap.parse_args())

    json_file = args['json_file']
    collection_selection_function = args['collection_selection_function']
    base_link = args['base_link']
    hand_link = args['hand_link']
    camera = args['camera']
    pattern = args['pattern']

    # Read dataset file
    dataset, json_file = loadResultsJSON(json_file, collection_selection_function)
    args['remove_partial_detections'] = True
    dataset = filterCollectionsFromDataset(dataset, args)
    
    dataset_ground_truth = deepcopy(dataset)  # make a copy before adding noise
    dataset_initial = deepcopy(dataset)  # store initial values

    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # We only need to get one collection because optimized transformations are static, which means they are the same for all collections. Let's select the first key in the dictionary and always get that transformation.
    selected_collection_key = list(dataset["collections"].keys())[0]
    print("Selected collection key is " + str(selected_collection_key))

    # ---------------------------------------
    # Verifications
    # ---------------------------------------

    # Check that the camera has rgb modality
    if not dataset['sensors'][args['camera']]['modality'] == 'rgb':
        atomError('Sensor ' + args['camera'] + ' is not of rgb modality.')

    # Check if the hand link is in the chain from the base to the pattern (since transforms involving the pattern aren't included in the transformation pool in the collections, it uses the pattern's parent link for the check)
    chain = getChain(from_frame=args['base_link'],
                     to_frame=dataset['calibration_config']['calibration_patterns'][args['pattern']]['parent_link'],
                     transform_pool=dataset['collections'][selected_collection_key]['transforms'])

    hand_frame_in_chain = False
    for transform in chain: 
        if args['hand_link'] == transform['parent'] or args['hand_link'] == transform['child']:
            hand_frame_in_chain = True
    
    if not hand_frame_in_chain:
        atomError('Selected hand link ' + Fore.BLUE + args['hand_link'] + Style.RESET_ALL +
                  ' does not belong to the chain from base ' + Fore.BLUE + args['base_link'] +
                  Style.RESET_ALL + ' to the camera ' +
                  dataset['calibration_config']['sensors'][args['camera']]['link'])

    # Check if the given hand link is not in the chain from base to camera (if it is, we're in an eye-in-hand configuration)
    chain = getChain(from_frame=args['base_link'],
                     to_frame=dataset['calibration_config']['sensors'][args['camera']]['link'],
                     transform_pool=dataset['collections'][selected_collection_key]['transforms'])


    hand_frame_in_chain = False
    for transform in chain:

        if args['hand_link'] == transform['parent'] or args['hand_link'] == transform['child']:
            hand_frame_in_chain = True

    if hand_frame_in_chain:
        atomError('Selected hand link ' + Fore.BLUE + args['hand_link'] + Style.RESET_ALL +
                  ' belongs to the chain from base ' + Fore.BLUE + args['base_link'] +
                  Style.RESET_ALL + ' to the camera ' +
                  dataset['calibration_config']['sensors'][args['camera']]['link'] + ', which indicates this system is not in an eye-to-hand configuration.')

    # Check the hand to pattern chain is composed only of fixed transforms
    # Since the transformation pool from a collection doesn't include the tf from the pattern link's parent to the pattern link, we must work with the parent. However, it might be the case that the hand link is the same as the pattern's parent link. In that case, it is known that the transform is fixed.
    
    if args['hand_link'] != dataset['calibration_config']['calibration_patterns'][args['pattern']]['parent_link']:
        
        chain = getChain(from_frame=args['hand_link'],
                        to_frame=dataset['calibration_config']['calibration_patterns'][args['pattern']]['parent_link'],
                        transform_pool=dataset['collections'][selected_collection_key]['transforms'])

        for transform in chain:
            if not dataset['transforms'][transform['key']]['type'] == 'fixed':
                atomError('Chain from hand link ' + Fore.BLUE + args['hand_link'] + Style.RESET_ALL +
                        ' to pattern link ' + Fore.BLUE +
                        dataset['calibration_config']['calibration_patterns'][args['pattern']]['link'] +
                        Style.RESET_ALL + ' contains non fixed transform ' + Fore.RED +
                        transform['key'] + Style.RESET_ALL + '. Cannot calibrate.')



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
    number_of_corners = int(nx) * int(ny)

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

        B = np.linalg.inv(tf_pattern2opticalframe) # Need to invert in the case of eye-to-hand 
        BB.append(B)


    # Z is the base to camera tf (b_T_c)
    b_T_c, h_T_p = li_calib(AA,BB)

    # Extract the transformation marked for calibration which is the
    # cp_T_cc, where cp (calibration parent) and cc (calibration child).
    # So we need to get cp_T_cc from the estimated b_T_c.
    # We use the following:
    # h_T_c = h_T_cp * cp_T_cc * cc_T_c
    # <=> cp_T_cc =  cp_T_h * h_T_c * c_T_cc

    calibration_parent = dataset['calibration_config']['sensors'][args['camera']]['parent_link']
    calibration_child = dataset['calibration_config']['sensors'][args['camera']]['child_link']

    cp_T_b = getTransform(from_frame=calibration_parent,
                          to_frame=args['base_link'],
                          transforms=dataset['collections'][selected_collection_key]['transforms'])

    c_T_cc = getTransform(from_frame=dataset['calibration_config']['sensors'][args['camera']]['link'],
                          to_frame=calibration_child,
                          transforms=dataset['collections'][selected_collection_key]['transforms'])

    cp_T_cc = cp_T_b @ b_T_c @ c_T_cc

    # Save to dataset
    # Since the transformation cp_T_cc is static we will save the same transform to all collections
    frame_key = generateKey(calibration_parent, calibration_child)

    quat = tf.transformations.quaternion_from_matrix(cp_T_cc)
    trans = cp_T_cc[0:3, 3].tolist()
    for collection_key, collection in dataset['collections'].items():
        dataset['collections'][collection_key]['transforms'][frame_key]['quat'] = quat
        dataset['collections'][collection_key]['transforms'][frame_key]['trans'] = trans

    if args['compare_to_ground_truth']:

        # --------------------------------------------------
        # Compare h_T_c hand to camera transform to ground truth
        # --------------------------------------------------
        b_T_c_ground_truth = getTransform(from_frame=args['base_link'],
                                          to_frame=dataset['calibration_config']['sensors'][args['camera']]['link'],
                                          transforms=dataset_ground_truth['collections'][selected_collection_key]['transforms'])
        print(Fore.GREEN + 'Ground Truth b_T_c=\n' + str(b_T_c_ground_truth))

        print('estimated b_T_c=\n' + str(b_T_c))

        translation_error, rotation_error, _, _, _, _, _, _ = compareTransforms(
            b_T_c, b_T_c_ground_truth)
        print('Etrans = ' + str(round(translation_error*1000, 3)) + ' (mm)')
        print('Erot = ' + str(round(rotation_error*180/math.pi, 3)) + ' (deg)')

        # --------------------------------------------------
        # Compare cp_T_cc calibration_parent_T_calibration_child to ground truth
        # --------------------------------------------------
        for sensor_key, sensor in dataset["sensors"].items():
            header = ['Transform', 'Description', 'Et0 [m]',
                      'Et [m]', 'Rrot0 [rad]', 'Erot [rad]']
            table = PrettyTable(header)

            transform_key = generateKey(
                sensor["calibration_parent"], sensor["calibration_child"])
            row = [transform_key, Fore.BLUE + sensor_key]

            transform_calibrated = dataset['collections'][selected_collection_key]['transforms'][transform_key]
            transform_ground_truth = dataset_ground_truth['collections'][
                selected_collection_key]['transforms'][transform_key]
            transform_initial = dataset_initial['collections'][selected_collection_key]['transforms'][transform_key]

            translation_error_1, rotation_error_1 = compareAtomTransforms(
                transform_initial, transform_ground_truth)
            translation_error_2, rotation_error_2 = compareAtomTransforms(
                transform_calibrated, transform_ground_truth)

            row.append(round(translation_error_1, 6))
            row.append(round(translation_error_2, 6))
            row.append(round(rotation_error_1, 6))
            row.append(round(rotation_error_2, 6))

            table.add_row(row)
            print(table)
 


if __name__ == '__main__':
    main()