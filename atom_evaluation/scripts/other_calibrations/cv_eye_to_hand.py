#!/usr/bin/env python3

"""
Handeye calibration from opencv. Eye to hand variant.
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import math
import os
import numpy as np
import argparse
import cv2
import tf
from colorama import Fore, Style
from copy import deepcopy
from numpy.linalg import inv
from prettytable import PrettyTable
from colorama import init as colorama_init

from atom_calibration.calibration.visualization import getCvImageFromCollectionSensor
from atom_core.atom import getChain, getTransform
from atom_core.dataset_io import filterCollectionsFromDataset, loadResultsJSON, saveAtomDataset
from atom_core.geometry import matrixToTranslationRotation, translationRotationToTransform, traslationRodriguesToTransform
from atom_core.naming import generateKey
from atom_core.transformations import compareTransforms
from atom_core.utilities import atomError, compareAtomTransforms

colorama_init(autoreset=True)
np.set_printoptions(precision=3, suppress=True)

# -------------------------------------------------------------------------------
#  FUNCTIONS
# -------------------------------------------------------------------------------


def main():

    # ---------------------------------------
    # Command line arguments
    # ---------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-c", "--camera", help="Camera sensor name.",
                    type=str, required=True)
    ap.add_argument("-p", "--pattern",
                    help="Pattern to be used for calibration.", type=str, required=True)
    ap.add_argument("-hl", "--hand_link",
                    help="Name of coordinate frame of the hand.", type=str, required=True)
    ap.add_argument("-bl", "--base_link",
                    help="Name of coordinate frame of the robot's base.", type=str, required=True)
    ap.add_argument("-ctgt", "--compare_to_ground_truth", action="store_true",
                    help="If the system being calibrated is simulated, directly compare the TFs to the ground truth.")
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=str,
                    help="A string to be evaluated into a lambda function that receives a collection name as input and "
                    "returns True or False to indicate if the collection should be loaded (and used in the "
                    "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
                    "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    ap.add_argument("-uic", "--use_incomplete_collections", action="store_true", default=False,
                    help="Remove any collection which does not have a detection for all sensors.", )
    ap.add_argument("-si", "--show_images", action="store_true",
                    default=False, help="shows images for each camera")
    ap.add_argument("-mn", "--method_name", required=False, default='tsai',
                    help="Hand eye method. One of ['tsai', 'park', 'horaud', 'andreff', 'daniilidis'].", type=str)

    args = vars(ap.parse_args())

    # ---------------------------------------
    # Dataset loading and preprocessing
    # ---------------------------------------
    dataset, _ = loadResultsJSON(args["json_file"],
                                 args["collection_selection_function"])

    # opencv can only process complete detections
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

    available_methods = ['tsai', 'park', 'horaud', 'andreff', 'daniilidis']
    if args['method_name'] not in available_methods:
        atomError('Unknown method. Select one from ' + str(available_methods))

    if args['method_name'] == 'tsai':
        method = cv2.CALIB_HAND_EYE_TSAI
    elif args['method_name'] == 'park':
        method = cv2.CALIB_HAND_EYE_PARK
    elif args['method_name'] == 'horaud':
        method = cv2.CALIB_HAND_EYE_HORAUD
    elif args['method_name'] == 'andreff ':
        method = cv2.CALIB_HAND_EYE_ANDREFF
    elif args['method_name'] == 'daniilidis':
        method = cv2.CALIB_HAND_EYE_DANIILIDIS

    # Check if the hand link is in the chain from the base to the pattern (since transforms involving the pattern aren't included in the transformation pool in the collections, it uses the pattern's parent link for the check)
    chain = getChain(from_frame=args['base_link'],
                     to_frame=dataset['calibration_config']['calibration_patterns'][args['pattern']]['parent_link'],
                     transform_pool=dataset['collections'][selected_collection_key]['transforms'])

    # chain is a list of dictionaries like this:
    # [{'parent': 'forearm_link', 'child': 'wrist_1_link', 'key': 'forearm_link-wrist_1_link'},
    #  {'parent': 'wrist_1_link', 'child': 'wrist_2_link', 'key': 'wrist_1_link-wrist_2_link'}, ... ]
    
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


    # ---------------------------------------
    # Pattern configuration
    # ---------------------------------------
    nx = dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                               ]['dimension']['x']
    ny = dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                               ]['dimension']['y']
    square = dataset['calibration_config']['calibration_patterns'][args['pattern']]['size']
    pts_3d = np.zeros((nx * ny, 3), np.float32)
    # set of coordinates (w.r.t. the pattern frame) of the corners
    pts_3d[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    number_of_corners = int(nx) * int(ny)

    # ---------------------------------------
    # --- Get intrinsic data for the sensor
    # ---------------------------------------
    # Source sensor
    K = np.zeros((3, 3), np.float32)
    D = np.zeros((5, 1), np.float32)
    K[0, :] = dataset['sensors'][args['camera']]['camera_info']['K'][0:3]
    K[1, :] = dataset['sensors'][args['camera']]['camera_info']['K'][3:6]
    K[2, :] = dataset['sensors'][args['camera']]['camera_info']['K'][6:9]
    D[:, 0] = dataset['sensors'][args['camera']]['camera_info']['D'][0:5]

    # ---------------------------------------
    # --- Create lists of transformations for hand eye calibration
    # ---------------------------------------
    # Hand eye calibration (in a eye-in-hand configuration) has 4 different transformations.
    #
    # robot-base to hand         b_T_h       (obtained from direct kinematics)
    # camera to pattern          c_T_p       (obtained through the detection of the known pattern)
    # hand to camera             h_T_c       (estimated by the calibration)
    # robot-base to pattern      b_T_p       (estimated by the calibration)
    # We will use this notation throughout the code.

    c_T_p_lst = []  # list of camera to pattern 4x4 transforms.
    b_T_h_lst = []  # list of base to hand 4x4 transforms.

    # Iterate all collections and create the lists of transforms c_T_p and b_T_h
    # NOTE: cannot test with a single collection. We need at least three. Check
    # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga41b1a8dd70eae371eba707d101729c36
    for collection_key, collection in dataset['collections'].items():

        # Pattern not detected by sensor in collection
        if not collection['labels'][args['pattern']][args['camera']]['detected']:
            continue

        # ---------------------------------------
        # Get c_T_p using pattern detection
        # ---------------------------------------
        # we will us the solve pnp function from opencv.
        # This requires a np array of the pattern corners 3d coordinates
        # and another np array of the correponding 2d pixels.
        # 3d coordinates were extracted previously, so lets go to 2d..
        pts_2d = np.ones((number_of_corners, 2), np.float32)
        for idx, point in enumerate(collection['labels'][args['pattern']][args['camera']]['idxs']):
            pts_2d[idx, 0] = point['x']
            pts_2d[idx, 1] = point['y']

        retval, rvec, tvec = cv2.solvePnP(pts_3d, pts_2d, K, D)
        if not retval:
            raise atomError('solvePnP failed.')

        # Convert to 4x4 transform and add to list
        c_T_p = traslationRodriguesToTransform(tvec, rvec)
        c_T_p_lst.append(c_T_p)

        # Visualize detections on image.
        if args['show_images']:
            image_gui = getCvImageFromCollectionSensor(
                collection_key, 'rgb_hand', dataset)

            # Four 3d points:  origin + axis tips
            axis_in_pattern = np.float32([[0, 0, 0],
                                          [0.2, 0, 0],
                                          [0, 0.2, 0],
                                          [0, 0, 0.2]])

            # Project to camera
            axis_in_camera, _ = cv2.projectPoints(axis_in_pattern, rvec, tvec,
                                                  K, D)

            # draw x axes
            origin = tuple(axis_in_camera[0][0].astype(int))
            x_tip = tuple(axis_in_camera[1][0].astype(int))
            y_tip = tuple(axis_in_camera[2][0].astype(int))
            z_tip = tuple(axis_in_camera[3][0].astype(int))
            image_gui = cv2.line(image_gui, origin, x_tip,
                                 (0, 0, 255), 5)  # opencv is BGR
            image_gui = cv2.line(image_gui, origin, y_tip, (0, 255, 0), 5)
            image_gui = cv2.line(image_gui, origin, z_tip, (255, 0, 0), 5)

            window_name = 'Collection ' + collection_key
            cv2.imshow(window_name, image_gui)
            cv2.waitKey(0)
            cv2.destroyWindow(window_name)

        # ---------------------------------------
        # Get b_T_h using direct kinematics
        # ---------------------------------------
        # We will use the atom builtin functions to get the transform directly
        b_T_h = getTransform(from_frame=args['base_link'],
                             to_frame=args['hand_link'],
                             transforms=collection['transforms'])
        b_T_h_lst.append(b_T_h)

    # ---------------------------------------
    # Run hand eye calibration
    # ---------------------------------------
    # We will use opencv's function calibrateHandEye()
    # https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
    # It requires a list of c_T_p and b_T_h transformations.
    # Transformations are represented by separate rotation (3x3) and translation (3x1) components, so we will have two lists per transformations.

    # NOTE instead of lists we could also use np arrays like this, same result.
    # n = len(c_T_p_lst)
    # c_T_p_Rs = np.zeros((n, 3, 3))
    # c_T_p_ts = np.zeros((n, 3, 1))
    # b_T_h_Rs = np.zeros((n, 3, 3))
    # b_T_h_ts = np.zeros((n, 3, 1))

    c_T_p_Rs = []  # list of rotations for c_T_p.
    c_T_p_ts = []  # list of translations for c_T_p
    b_T_h_Rs = []  # list of rotations for b_T_h
    b_T_h_ts = []  # list of translations for b_T_h

    for c_T_p, b_T_h in zip(c_T_p_lst, b_T_h_lst):
        # --- c_T_p    camera to pattern
        c_T_p_t, c_T_p_R = matrixToTranslationRotation(c_T_p)
        c_T_p_Rs.append(c_T_p_R)
        c_T_p_ts.append(c_T_p_t)

        # --- b_T_h    base to hand
        b_T_h_t, b_T_h_R = matrixToTranslationRotation(b_T_h)
        b_T_h_Rs.append(b_T_h_R)
        b_T_h_ts.append(b_T_h_t)

    # ---------------------------------------
    # Run hand eye calibration using calibrateHandEye
    # ---------------------------------------

    # In an eye-to-hand configuration, it returns b_T_c instead of h_T_c
    b_T_c_R, b_T_c_t = cv2.calibrateHandEye(b_T_h_Rs,
                                            b_T_h_ts,
                                            c_T_p_Rs,
                                            c_T_p_ts,
                                            method=method)

    # Rotations out of calibrateRobotWorldHandEye are 3x3
    b_T_c = translationRotationToTransform(b_T_c_t, b_T_c_R)
    

    # Extract the transformation marked for calibration which is the
    # cp_T_cc, where cp (calibration parent) and cc (calibration child).
    # So we need to get cp_T_cc from the estimated h_T_c.
    # We use the following:
    # b_T_c = b_T_cp * cp_T_cc * cc_T_c
    # <=> cp_T_cc =  (b_T_cp)-1 * b_T_c * (cc_T_c)-1

    calibration_parent = dataset['calibration_config']['sensors'][args['camera']]['parent_link']
    calibration_child = dataset['calibration_config']['sensors'][args['camera']]['child_link']

    b_T_cp = getTransform(from_frame=args['base_link'],
                          to_frame=calibration_parent,
                          transforms=dataset['collections'][selected_collection_key]['transforms'])

    cc_T_c = getTransform(from_frame=calibration_child,
                          to_frame=dataset['calibration_config']['sensors'][args['camera']]['link'],
                          transforms=dataset['collections'][selected_collection_key]['transforms'])

    cp_T_cc = inv(b_T_cp) @ b_T_c @ inv(cc_T_c)

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
        # Compare b_T_c base to camera transform to ground truth
        # --------------------------------------------------
        b_T_c_ground_truth = getTransform(from_frame=args['base_link'],
                                          to_frame=dataset['calibration_config']['sensors'][args['camera']]['link'],
                                          transforms=dataset['collections'][selected_collection_key]['transforms'])
        print(Fore.GREEN + 'Ground Truth b_T_c=\n' + str(b_T_c_ground_truth))

        print('estimated b_T_c=\n' + str(b_T_c))

        translation_error, rotation_error, _, _, _, _, _, _ = compareTransforms(
            b_T_c, b_T_c_ground_truth)
        print('Etrans = ' + str(round(translation_error*1000, 3)) + ' (m)')
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

    # Save results to an atom dataset
    filename_results_json = os.path.dirname(
        args['json_file']) + '/hand_eye_' + args['method_name'] + '_' + args['camera'] + '.json'
    saveAtomDataset(filename_results_json, dataset)


if __name__ == '__main__':
    main()
