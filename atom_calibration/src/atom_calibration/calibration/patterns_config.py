#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

import math
import pprint
from copy import deepcopy

import atom_core.atom

# 3rd-party
import numpy as np
import cv2

# stdlib
from colorama import Fore, Style
from tf import transformations
from atom_core.geometry import traslationRodriguesToTransform
from atom_calibration.collect import patterns as opencv_patterns

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def sampleLineSegment(p0, p1, step):
    norm = math.sqrt((p1['x'] - p0['x']) ** 2 + (p1['y'] - p0['y']) ** 2)
    n = round(norm / step)
    vector_x = p1['x'] - p0['x']
    vector_y = p1['y'] - p0['y']
    pts = []
    for alfa in np.linspace(0, 1, num=n, endpoint=True, retstep=False, dtype=float):
        x = p0['x'] + vector_x * alfa
        y = p0['y'] + vector_y * alfa
        pts.append({'x': x, 'y': y})
    return pts


def createPatternLabels(args, dataset, step=0.02):
    """
    Creates the necessary data related to the chessboard calibration pattern
    :return: a dataset_chessboard dictionary
    """

    # TODO #738 is this really operational right now?
#     if 'patterns' in dataset:  # If we have a patterns that means that the pattern poses were already found and written to the dataset. In that case we use the poses that exist.
#         print('Dataset already contains patterns.' + Fore.BLUE +
#               ' Will skip generation of pattern labels and retrieve world to pattern transforms from the dataset.' +
#               Style.RESET_ALL)
#
#         patterns = dataset['patterns']
#
#         for collection_key, collection in dataset['collections'].items():
#             parent = dataset["calibration_config"]["calibration_pattern"]["parent_link"]
#             child = dataset["calibration_config"]["calibration_pattern"]["link"]
#             transform_key = atom_core.naming.generateKey(parent, child)
#
#             # transformation already in dataset, use it
#             if transform_key in dataset['collections'][collection_key]['transforms']:
#                 trans = dataset['collections'][collection_key]['transforms'][transform_key]['trans']
#                 quat = dataset['collections'][collection_key]['transforms'][transform_key]['quat']
#                 patterns['transforms_initial'][str(collection_key)] = \
#                     {'detected': True, 'sensor': 'from_dataset', 'parent': parent, 'child': child,
#                         'trans': trans, 'quat': quat,
#                      }
#         return patterns

    patterns_dict = {}
    for pattern_key, calibration_pattern in dataset['calibration_config']['calibration_patterns'].items():

        nx = calibration_pattern['dimension']['x']
        ny = calibration_pattern['dimension']['y']
        square = calibration_pattern['size']
        # Border can be a scalar or {'x': ..., 'y': ...}
        if type(calibration_pattern['border_size']) is dict:
            border_x = calibration_pattern['border_size']['x']
            border_y = calibration_pattern['border_size']['y']
        else:
            border_x = border_y = calibration_pattern['border_size']

        pattern_dict = {  # All coordinates in the pattern's local coordinate system. Since z=0 for all points, it is omitted.
            # [{'idx': 0, 'x': 3, 'y': 4}, ..., ] # Pattern's visual markers
            'corners': [],
            'frame': {'corners': {'top_left': {'x': 0, 'y': 0},  # Physical outer boundaries of the pattern
                                  'top_right': {'x': 0, 'y': 0},
                                  'bottom_right': {'x': 0, 'y': 0},
                                  'bottom_left': {'x': 0, 'y': 0}},
                      'lines_sampled': {'top': [],  # [{'x':0, 'y':0}]
                                        'bottom': [],  # [{'x':0, 'y':0}]
                                        'left': [],  # [{'x':0, 'y':0}]
                                        'right': []}  # [{'x':0, 'y':0}]
                      },
            # Transitions from black to white squares. Just a set of sampled points
            'transitions': {'vertical': [],  # [{'x': 3, 'y': 4}, ..., {'x': 30, 'y': 40}]},
                            'horizontal': []},  # [{'x': 3, 'y': 4}, ..., {'x': 30, 'y': 40}]},
            'transforms_initial': {},  # {'collection_key': {'trans': ..., 'quat': 4}, ...}
        }

        if calibration_pattern['pattern_type'] == 'chessboard':
            # Chessboard: Origin on top left corner, X left to right, Y top to bottom

            # ---------------- Corners ----------------
            # idx left to right, top to bottom
            idx = 0
            for row in range(0, calibration_pattern['dimension']['y']):
                for col in range(0, calibration_pattern['dimension']['x']):
                    pattern_dict['corners'].append({'id': idx, 'x': col * square, 'y': row * square})
                    idx += 1

            # ---------------- Frame ----------------
            # Corners
            pattern_dict['frame']['corners']['top_left'] = {
                'x': -square - border_x, 'y': -square - border_y}
            pattern_dict['frame']['corners']['top_right'] = {
                'x': nx * square + border_x, 'y': -square - border_y}
            pattern_dict['frame']['corners']['bottom_right'] = {
                'x': nx * square + border_x, 'y': ny * square + border_y}
            pattern_dict['frame']['corners']['bottom_left'] = {
                'x': -square - border_x, 'y': ny * square + border_y}

            # Lines sampled
            pattern_dict['frame']['lines_sampled']['top'] = sampleLineSegment(
                pattern_dict['frame']['corners']['top_left'], pattern_dict['frame']['corners']['top_right'], step)
            pattern_dict['frame']['lines_sampled']['bottom'] = sampleLineSegment(
                pattern_dict['frame']['corners']['bottom_left'], pattern_dict['frame']['corners']['bottom_right'], step)
            pattern_dict['frame']['lines_sampled']['left'] = sampleLineSegment(pattern_dict['frame']['corners']['top_left'],
                                                                               pattern_dict['frame']['corners']['bottom_left'],
                                                                               step)
            pattern_dict['frame']['lines_sampled']['right'] = sampleLineSegment(
                pattern_dict['frame']['corners']['top_right'], pattern_dict['frame']['corners']['bottom_right'], step)

            # -------------- Transitions ----------------
            # vertical
            for col in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['x']):
                p0 = {'x': col * square, 'y': 0}
                p1 = {'x': col * square, 'y': (ny - 1) * square}
                pts = sampleLineSegment(p0, p1, step)
                pattern_dict['transitions']['vertical'].extend(pts)

            # horizontal
            for row in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['y']):
                p0 = {'x': 0, 'y': row * square}
                p1 = {'x': (nx - 1) * square, 'y': row * square}
                pts = sampleLineSegment(p0, p1, step)
                pattern_dict['transitions']['horizontal'].extend(pts)

            # pp = pprint.PrettyPrinter(indent=4)
            # pp.pprint(patterns)
            # exit(0)

        elif calibration_pattern['pattern_type'] == 'charuco':
            # Charuco: Origin on bottom left corner, X left to right, Y bottom to top

            # ---------------- Corners ----------------
            # idx left to right, bottom to top
            idx = 0
            for row in range(0, calibration_pattern['dimension']['y']):
                for col in range(0, calibration_pattern['dimension']['x']):
                    pattern_dict['corners'].append(
                        {'id': idx, 'x': square + col * square, 'y': square + row * square})
                    idx += 1

            # ---------------- Frame ----------------
            # Corners
            pattern_dict['frame']['corners']['top_left'] = {
                'x': -border_x, 'y': - border_y}
            pattern_dict['frame']['corners']['top_right'] = {
                'x': square + nx * square + border_x, 'y': -border_y}
            pattern_dict['frame']['corners']['bottom_right'] = {
                'x': square + nx * square + border_x, 'y': ny * square + border_y + square}
            pattern_dict['frame']['corners']['bottom_left'] = {
                'x': - border_x, 'y': ny * square + border_y + square}

            # Lines sampled
            pattern_dict['frame']['lines_sampled']['top'] = sampleLineSegment(
                pattern_dict['frame']['corners']['top_left'], pattern_dict['frame']['corners']['top_right'], step)
            pattern_dict['frame']['lines_sampled']['bottom'] = sampleLineSegment(
                pattern_dict['frame']['corners']['bottom_left'], pattern_dict['frame']['corners']['bottom_right'], step)
            pattern_dict['frame']['lines_sampled']['left'] = sampleLineSegment(pattern_dict['frame']['corners']['top_left'],
                                                                               pattern_dict['frame']['corners']['bottom_left'],
                                                                               step)
            pattern_dict['frame']['lines_sampled']['right'] = sampleLineSegment(
                pattern_dict['frame']['corners']['top_right'], pattern_dict['frame']['corners']['bottom_right'], step)

            # -------------- Transitions ----------------
            # vertical
            for col in range(0, calibration_pattern['dimension']['x']):
                p0 = {'x': col * square, 'y': 0}
                p1 = {'x': col * square, 'y': (ny - 1) * square}
                pts = sampleLineSegment(p0, p1, step)
                pattern_dict['transitions']['vertical'].extend(pts)

            # horizontal
            for row in range(0, calibration_pattern['dimension']['y']):
                p0 = {'x': 0, 'y': row * square}
                p1 = {'x': (nx - 1) * square, 'y': row * square}
                pts = sampleLineSegment(p0, p1, step)
                pattern_dict['transitions']['horizontal'].extend(pts)
        else:
            raise ValueError(
                'Unknown pattern type: ' + calibration_pattern['pattern_type'])

        # -----------------------------------
        # Create first guess for pattern pose
        # -----------------------------------
        config = dataset['calibration_config']
        # TODO only works for first pattern
        size = {'x': calibration_pattern['dimension']['x'],
                'y': calibration_pattern['dimension']['y']}
        length = calibration_pattern['size']
        inner_length = calibration_pattern['inner_size']
        dictionary = calibration_pattern['dictionary']

        opencv_pattern = opencv_patterns.CharucoPattern(size, length, inner_length, dictionary)

        for collection_key, collection in dataset['collections'].items():
            flg_detected_pattern = False
            pattern_dict['transforms_initial'][str(collection_key)] = {'detected': False}  # by default no detection

            # TODO #740 now sure why this sorting is here so I am removing it
            # for sensor_key, sensor in sorted(list(dataset['sensors'].items()), key=lambda x: x[0].lower(), reverse=True):
            for sensor_key, sensor in dataset['sensors'].items():

                # if chessboard not detected by sensor in collection
                if not collection['labels'][pattern_key][sensor_key]['detected']:
                    print('Collection ' + str(collection_key) +
                          ' is partial: Pattern not detected by sensor ' + str(sensor_key))
                    continue

                # change accordingly to the first camera to give chessboard first poses
                if sensor['modality'] == 'rgb':
                    K = np.ndarray((3, 3), dtype=float, buffer=np.array(
                        sensor['camera_info']['K']))
                    D = np.ndarray((5, 1), dtype=float, buffer=np.array(
                        sensor['camera_info']['D']))

                    # TODO should we not read these from the dictionary?
                    objp = np.zeros((nx * ny, 3), np.float32)
                    objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

                    # Build a numpy array with the charuco corners
                    corners = np.zeros(
                        (len(collection['labels'][pattern_key][sensor_key]['idxs']), 1, 2), dtype=float)
                    ids = list(
                        range(0, len(collection['labels'][pattern_key][sensor_key]['idxs'])))
                    for idx, point in enumerate(collection['labels'][pattern_key][sensor_key]['idxs']):
                        corners[idx, 0, 0] = point['x']
                        corners[idx, 0, 1] = point['y']
                        ids[idx] = point['id']

                    # Find pose of the camera w.r.t the chessboard
                    np_ids = np.array(ids, dtype=int)
                    rvecs, tvecs = None, None

                    # TODO only works for first pattern
                    if config['calibration_patterns'][pattern_key]['pattern_type'] == 'charuco':
                        _, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(np.array(corners, dtype=np.float32),
                                                                             np_ids, opencv_pattern.board,
                                                                             K, D, rvecs, tvecs)
                    else:
                        _, rvecs, tvecs = cv2.solvePnP(
                            objp[ids], np.array(corners, dtype=np.float32), K, D)

                    # Compute the pose of he chessboard w.r.t the pattern parent link
                    root_T_sensor = atom_core.atom.getTransform(
                        calibration_pattern['parent_link'],
                        sensor['camera_info']['header']['frame_id'], collection['transforms'])

                    sensor_T_chessboard = traslationRodriguesToTransform(
                        tvecs, rvecs)
                    root_T_chessboard = np.dot(root_T_sensor, sensor_T_chessboard)
                    T = deepcopy(root_T_chessboard)
                    T[0:3, 3] = 0  # remove translation component from 4x4 matrix

                    # print('Creating first guess for collection ' + collection_key + ' using sensor ' + sensor_key)

                    parent = calibration_pattern['parent_link']
                    child = calibration_pattern['link']
                    pattern_dict['transforms_initial'][
                        str(collection_key)] = {
                        'detected': True, 'sensor': sensor_key, 'parent': parent, 'child': child,
                        'trans': list(root_T_chessboard[0: 3, 3]),
                        'quat': list(transformations.quaternion_from_matrix(T)), }

                    flg_detected_pattern = True
                    break  # don't search for this collection's chessboard on anymore sensors

        if not flg_detected_pattern:  # Abort when the chessboard is not detected by any camera on this collection
            raise ValueError('Collection ' + collection_key +
                             ' could not find pattern.')

        patterns_dict[pattern_key] = pattern_dict  # add this pattern to the patterns dict

    return patterns_dict
