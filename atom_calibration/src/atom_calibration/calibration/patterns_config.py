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
from OptimizationUtils import utilities

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def sampleLineSegment(p0, p1, step):
    norm = math.sqrt((p1['x'] - p0['x']) ** 2 + (p1['y'] - p0['y']) ** 2)
    n = round(norm / step)
    vector_x = p1['x'] - p0['x']
    vector_y = p1['y'] - p0['y']
    pts = []
    for alfa in np.linspace(0, 1, num=n, endpoint=True, retstep=False, dtype=np.float):
        x = p0['x'] + vector_x * alfa
        y = p0['y'] + vector_y * alfa
        pts.append({'x': x, 'y': y})
    return pts


def createPatternLabels(args, dataset, step=0.02):
    """
    Creates the necessary data related to the chessboard calibration pattern
    :return: a dataset_chessboard dictionaryint((((args['chess_num_y'] * factor) - 1) * n) + 1) * (args['chess_num_x'] * factor)
    """

    if 'patterns' in dataset:  # If we have a patterns that means that the pattern poses were already found and written to the dataset. In that case we use the poses that exist.
        print('Dataset already contains patterns.' + Fore.BLUE +
              ' Will skip generation of pattern labels and retrieve world to pattern transforms from the dataset.' +
              Style.RESET_ALL)

        patterns = dataset['patterns']

        for collection_key, collection in dataset['collections'].items():
            parent = dataset["calibration_config"]["calibration_pattern"]["parent_link"]
            child = dataset["calibration_config"]["calibration_pattern"]["link"]
            transform_key = atom_core.naming.generateKey(parent, child)

            # transformation already in dataset, use it
            if transform_key in dataset['collections'][collection_key]['transforms']:
                trans = dataset['collections'][collection_key]['transforms'][transform_key]['trans']
                quat = dataset['collections'][collection_key]['transforms'][transform_key]['quat']
                patterns['transforms_initial'][str(collection_key)] = \
                    {'detected': True, 'sensor': 'from_dataset', 'parent': parent, 'child': child,
                        'trans': trans, 'quat': quat,
                     }
        return patterns

    nx = dataset['calibration_config']['calibration_pattern']['dimension']['x']
    ny = dataset['calibration_config']['calibration_pattern']['dimension']['y']
    square = dataset['calibration_config']['calibration_pattern']['size']
    # Border can be a scalar or {'x': ..., 'y': ...}
    if type(dataset['calibration_config']['calibration_pattern']['border_size']) is dict:
        border_x = dataset['calibration_config']['calibration_pattern']['border_size']['x']
        border_y = dataset['calibration_config']['calibration_pattern']['border_size']['y']
    else:
        border_x = border_y = dataset['calibration_config']['calibration_pattern']['border_size']

    patterns = {  # All coordinates in the pattern's local coordinate system. Since z=0 for all points, it is omitted.
        'corners': [],  # [{'idx': 0, 'x': 3, 'y': 4}, ..., ] # Pattern's visual markers
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

    if dataset['calibration_config']['calibration_pattern']['pattern_type'] == 'chessboard':
        # Chessboard: Origin on top left corner, X left to right, Y top to bottom

        # ---------------- Corners ----------------
        # idx left to right, top to bottom
        idx = 0
        for row in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['y']):
            for col in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['x']):
                patterns['corners'].append({'id': idx, 'x': col * square, 'y': row * square})
                idx += 1

        # ---------------- Frame ----------------
        # Corners
        patterns['frame']['corners']['top_left'] = {'x': -square - border_x, 'y': -square - border_y}
        patterns['frame']['corners']['top_right'] = {'x': nx * square + border_x, 'y': -square - border_y}
        patterns['frame']['corners']['bottom_right'] = {'x': nx * square + border_x, 'y': ny * square + border_y}
        patterns['frame']['corners']['bottom_left'] = {'x': -square - border_x, 'y': ny * square + border_y}

        # Lines sampled
        patterns['frame']['lines_sampled']['top'] = sampleLineSegment(
            patterns['frame']['corners']['top_left'], patterns['frame']['corners']['top_right'], step)
        patterns['frame']['lines_sampled']['bottom'] = sampleLineSegment(
            patterns['frame']['corners']['bottom_left'], patterns['frame']['corners']['bottom_right'], step)
        patterns['frame']['lines_sampled']['left'] = sampleLineSegment(patterns['frame']['corners']['top_left'],
                                                                       patterns['frame']['corners']['bottom_left'],
                                                                       step)
        patterns['frame']['lines_sampled']['right'] = sampleLineSegment(
            patterns['frame']['corners']['top_right'], patterns['frame']['corners']['bottom_right'], step)

        # -------------- Transitions ----------------
        # vertical
        for col in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['x']):
            p0 = {'x': col * square, 'y': 0}
            p1 = {'x': col * square, 'y': (ny - 1) * square}
            pts = sampleLineSegment(p0, p1, step)
            patterns['transitions']['vertical'].extend(pts)

        # horizontal
        for row in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['y']):
            p0 = {'x': 0, 'y': row * square}
            p1 = {'x': (nx - 1) * square, 'y': row * square}
            pts = sampleLineSegment(p0, p1, step)
            patterns['transitions']['horizontal'].extend(pts)

        # pp = pprint.PrettyPrinter(indent=4)
        # pp.pprint(patterns)
        # exit(0)

    elif dataset['calibration_config']['calibration_pattern']['pattern_type'] == 'charuco':
        # Charuco: Origin on bottom left corner, X left to right, Y bottom to top

        # ---------------- Corners ----------------
        # idx left to right, bottom to top
        idx = 0
        for row in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['y']):
            for col in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['x']):
                patterns['corners'].append({'id': idx, 'x': col * square, 'y': row * square})
                idx += 1

        # ---------------- Frame ----------------
        # Corners
        patterns['frame']['corners']['top_left'] = {'x': -square - border_x, 'y': -square - border_y}
        patterns['frame']['corners']['top_right'] = {'x': nx * square + border_x, 'y': -square - border_y}
        patterns['frame']['corners']['bottom_right'] = {'x': nx * square + border_x, 'y': ny * square + border_y}
        patterns['frame']['corners']['bottom_left'] = {'x': -square - border_x, 'y': ny * square + border_y}

        # Lines sampled
        patterns['frame']['lines_sampled']['top'] = sampleLineSegment(
            patterns['frame']['corners']['top_left'], patterns['frame']['corners']['top_right'], step)
        patterns['frame']['lines_sampled']['bottom'] = sampleLineSegment(
            patterns['frame']['corners']['bottom_left'], patterns['frame']['corners']['bottom_right'], step)
        patterns['frame']['lines_sampled']['left'] = sampleLineSegment(patterns['frame']['corners']['top_left'],
                                                                       patterns['frame']['corners']['bottom_left'],
                                                                       step)
        patterns['frame']['lines_sampled']['right'] = sampleLineSegment(
            patterns['frame']['corners']['top_right'], patterns['frame']['corners']['bottom_right'], step)

        # -------------- Transitions ----------------
        # vertical
        for col in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['x']):
            p0 = {'x': col * square, 'y': 0}
            p1 = {'x': col * square, 'y': (ny - 1) * square}
            pts = sampleLineSegment(p0, p1, step)
            patterns['transitions']['vertical'].extend(pts)

        # horizontal
        for row in range(0, dataset['calibration_config']['calibration_pattern']['dimension']['y']):
            p0 = {'x': 0, 'y': row * square}
            p1 = {'x': (nx - 1) * square, 'y': row * square}
            pts = sampleLineSegment(p0, p1, step)
            patterns['transitions']['horizontal'].extend(pts)
    else:
        raise ValueError(
            'Unknown pattern type: ' + dataset['calibration_config']['calibration_pattern']['pattern_type'])

    # -----------------------------------
    # Create first guess for pattern pose
    # -----------------------------------
    for collection_key, collection in dataset['collections'].items():
        flg_detected_pattern = False
        patterns['transforms_initial'][str(collection_key)] = {'detected': False}  # by default no detection

        for sensor_key, sensor in sorted(list(dataset['sensors'].items()), key=lambda x: x[0].lower(), reverse=True):
            # for sensor_key, sensor in dataset['sensors'].items():

            if not collection['labels'][sensor_key]['detected']:  # if chessboard not detected by sensor in collection
                print('Collection ' + str(collection_key) + ' is partial: Pattern not detected by sensor ' + str(sensor_key))
                continue

            # change accordingly to the first camera to give chessboard first poses
            if sensor['modality'] == 'rgb':
                K = np.ndarray((3, 3), dtype=np.float, buffer=np.array(sensor['camera_info']['K']))
                D = np.ndarray((5, 1), dtype=np.float, buffer=np.array(sensor['camera_info']['D']))

                # TODO should we not read these from the dictionary?
                objp = np.zeros((nx * ny, 3), np.float32)
                objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
                # Build a numpy array with the chessboard corners
                corners = np.zeros((len(collection['labels'][sensor_key]['idxs']), 1, 2), dtype=np.float)
                ids = list(range(0, len(collection['labels'][sensor_key]['idxs'])))
                for idx, point in enumerate(collection['labels'][sensor_key]['idxs']):
                    corners[idx, 0, 0] = point['x']
                    corners[idx, 0, 1] = point['y']
                    ids[idx] = point['id']

                # Find pose of the camera w.r.t the chessboard
                ret, rvecs, tvecs = cv2.solvePnP(objp[ids], np.array(corners, dtype=np.float32), K, D)

                # Compute the pose of he chessboard w.r.t the pattern parent link
                root_T_sensor = atom_core.atom.getTransform(
                    dataset['calibration_config']['calibration_pattern']['parent_link'],
                    sensor['camera_info']['header']['frame_id'], collection['transforms'])

                sensor_T_chessboard = utilities.traslationRodriguesToTransform(tvecs, rvecs)
                root_T_chessboard = np.dot(root_T_sensor, sensor_T_chessboard)
                T = deepcopy(root_T_chessboard)
                T[0:3, 3] = 0  # remove translation component from 4x4 matrix

                # print('Creating first guess for collection ' + collection_key + ' using sensor ' + sensor_key)

                parent = dataset['calibration_config']['calibration_pattern']['parent_link']
                child = dataset['calibration_config']['calibration_pattern']['link']
                patterns['transforms_initial'][
                    str(collection_key)] = {
                    'detected': True, 'sensor': sensor_key, 'parent': parent, 'child': child,
                    'trans': list(root_T_chessboard[0: 3, 3]),
                    'quat': list(transformations.quaternion_from_matrix(T)), }

                flg_detected_pattern = True
                break  # don't search for this collection's chessboard on anymore sensors

    if not flg_detected_pattern:  # Abort when the chessboard is not detected by any camera on this collection
        raise ValueError('Collection ' + collection_key + ' could not find pattern.')

    return patterns

    # step = 0.01
    #
    # # Limit Points ----------------
    # # Miguel's way: Create limit_points (all points defined in 2D, no z, not homogeneous) in the chessboard ref frame.
    # pts = [[], []]
    #
    # # Left vertical line
    # x = -square
    # y0 = -square
    # y1 = ny * square
    # for y in list(np.linspace(y0, y1, num=int(abs(y1 - y0) / step), dtype=np.float)):
    #     pts[0].append(x), pts[1].append(y)
    #
    # # Right vertical line
    # x = nx * square
    # y0 = -square
    # y1 = ny * square
    # for y in list(np.linspace(y0, y1, num=int(abs(y1 - y0) / step), dtype=np.float)):
    #     pts[0].append(x), pts[1].append(y)
    #
    # # Top horizontal line
    # # x0 = -square
    # # x1 = nx * square
    # # y = -square
    # # for x in list(np.linspace(x0, x1, num=int(abs(x1 - x0) / step), dtype=np.float)):
    # #     pts[0].append(x), pts[1].append(y)
    #
    # # Bottom horizontal line
    # # x0 = -square
    # # x1 = nx * square
    # # y = ny * square
    # # for x in list(np.linspace(x0, x1, num=int(abs(x1 - x0) / step), dtype=np.float)):
    # #     pts[0].append(x), pts[1].append(y)
    #
    # pts = np.array(pts, np.float)  # convert list to numpy array
    # pts = np.vstack((pts, np.zeros((1, pts.shape[1]))))  # add z = 0 coordinates to all points
    # pts = np.vstack((pts, np.ones((1, pts.shape[1]))))  # homogenize all points
    #
    #
    # dataset_chessboards['limit_points'] = pts
    #
    # # Limit Points ----------------
    # # Miguel's way: Create inner_points (all points defined in 2D, no z, not homogeneous) in the chessboard ref frame.
    # pts = [[], []]
    #
    # # Vertical lines
    # for x in [x * square for x in range(0, nx)]:
    #     y0 = 0
    #     y1 = (ny - 1) * square
    #     for y in list(np.linspace(y0, y1, num=int(abs(y1 - y0) / step), dtype=np.float)):
    #         pts[0].append(x), pts[1].append(y)
    #
    # # Horizontal lines
    # # for y in [y * square for y in range(0, ny)]:
    # #     x0 = 0
    # #     x1 = (nx-1) * square
    # #     for x in list(np.linspace(x0, x1, num=int(abs(x1 - x0) / step), dtype=np.float)):
    # #         pts[0].append(x), pts[1].append(y)
    #
    # pts = np.array(pts, np.float)  # convert list to numpy array
    # pts = np.vstack((pts, np.zeros((1, pts.shape[1]))))  # add z = 0 coordinates to all points
    # pts = np.vstack((pts, np.ones((1, pts.shape[1]))))  # homogenize all points
    # dataset_chessboards['inner_points'] = pts
    #
    # # TODO limit points number should be a parsed argument
    # n = 10
    # factor = round(1.)
    # num_pts = int((nx * factor) * (ny * factor))
    # num_l_pts = int((nx * factor) * 2 * n) + int((ny * factor) * 2 * n) + (4 * n)
    # num_i_pts = int(((nx * factor) - 1) * (n - 1)) * (ny * factor) + int(
    #     ((ny * factor) - 1) * (n - 1)) * (nx * factor) + num_pts
    # chessboard_evaluation_points = np.zeros((4, num_pts), np.float32)
    # chessboard_limit_points = np.zeros((4, int(num_l_pts)), np.float32)
    # chessboard_inner_points = np.zeros((4, int(num_i_pts)), np.float32)
    # step_x = nx * square / (nx * factor)
    # step_y = ny * square / (ny * factor)
    #
    # counter = 0
    # l_counter = 0
    # i_counter = 0
    # # TODO @afonsocastro should put this more synthesized
    # for idx_y in range(0, int(ny * factor)):
    #     y = idx_y * step_y
    #     for idx_x in range(0, int(nx * factor)):
    #         x = idx_x * step_x
    #         chessboard_evaluation_points[0, counter] = x
    #         chessboard_evaluation_points[1, counter] = y
    #         chessboard_evaluation_points[2, counter] = 0
    #         chessboard_evaluation_points[3, counter] = 1
    #         counter += 1
    #         # if idx_x != (int(nx * factor) - 1):
    #         #     for i in range(0, n):
    #         #         chessboard_inner_points[0, i_counter] = x + (i * (step_x / n))
    #         #         chessboard_inner_points[1, i_counter] = y
    #         #         chessboard_inner_points[2, i_counter] = 0
    #         #         chessboard_inner_points[3, i_counter] = 1
    #         #         i_counter += 1
    #         # else:
    #         #     chessboard_inner_points[0, i_counter] = x
    #         #     chessboard_inner_points[1, i_counter] = y
    #         #     chessboard_inner_points[2, i_counter] = 0
    #         #     chessboard_inner_points[3, i_counter] = 1
    #         #     i_counter += 1
    #
    #         if idx_y != (int(ny * factor) - 1):
    #             for i in range(1, n):
    #                 chessboard_inner_points[0, i_counter] = x
    #                 chessboard_inner_points[1, i_counter] = y + (i * (step_y / n))
    #                 chessboard_inner_points[2, i_counter] = 0
    #                 chessboard_inner_points[3, i_counter] = 1
    #                 i_counter += 1
    #
    #         # if idx_y == 0:
    #         #     for i in range(0, n):
    #         #         chessboard_limit_points[0, l_counter] = x - ((n - i) * (step_x / n))
    #         #         chessboard_limit_points[1, l_counter] = y - step_y
    #         #         chessboard_limit_points[2, l_counter] = 0
    #         #         chessboard_limit_points[3, l_counter] = 1
    #         #         l_counter += 1
    #         #
    #         #     if idx_x == (int(nx * factor) - 1):
    #         #         for i in range(n, 0, -1):
    #         #             chessboard_limit_points[0, l_counter] = x + ((n - i) * (step_x / n))
    #         #             chessboard_limit_points[1, l_counter] = y - step_y
    #         #             chessboard_limit_points[2, l_counter] = 0
    #         #             chessboard_limit_points[3, l_counter] = 1
    #         #             l_counter += 1
    #
    #         if idx_x == (int(nx * factor) - 1):
    #             for i in range(0, n):
    #                 chessboard_limit_points[0, l_counter] = x + step_x
    #                 chessboard_limit_points[1, l_counter] = y - ((n - i) * (step_y / n))
    #                 chessboard_limit_points[2, l_counter] = 0
    #                 chessboard_limit_points[3, l_counter] = 1
    #                 l_counter += 1
    #
    #             if idx_y == (int(ny * factor) - 1):
    #                 for i in range(n, 0, -1):
    #                     chessboard_limit_points[0, l_counter] = x + step_x
    #                     chessboard_limit_points[1, l_counter] = y + ((n - i) * (step_y / n))
    #                     chessboard_limit_points[2, l_counter] = 0
    #                     chessboard_limit_points[3, l_counter] = 1
    #                     l_counter += 1
    #
    # for idx_y in range(0, int(ny * factor)):
    #     idx_y = abs(idx_y - (int(ny * factor) - 1))
    #     y = idx_y * step_y
    #
    #     for idx_x in range(0, int(nx * factor)):
    #         idx_x = abs(idx_x - (int(nx * factor) - 1))
    #         x = idx_x * step_x
    #
    #         # if idx_y == (int(ny * factor) - 1):
    #         #     for i in range(0, n):
    #         #         chessboard_limit_points[0, l_counter] = x + ((n - i) * (step_x / n))
    #         #         chessboard_limit_points[1, l_counter] = y + step_y
    #         #         chessboard_limit_points[2, l_counter] = 0
    #         #         chessboard_limit_points[3, l_counter] = 1
    #         #         l_counter += 1
    #         #
    #         #     if idx_x == 0:
    #         #         for i in range(n, 0, -1):
    #         #             chessboard_limit_points[0, l_counter] = x - ((n - i) * (step_x / n))
    #         #             chessboard_limit_points[1, l_counter] = y + step_y
    #         #             chessboard_limit_points[2, l_counter] = 0
    #         #             chessboard_limit_points[3, l_counter] = 1
    #         #             l_counter += 1
    #
    #         if idx_x == 0:
    #             for i in range(0, n):
    #                 chessboard_limit_points[0, l_counter] = x - step_x
    #                 chessboard_limit_points[1, l_counter] = y + ((n - i) * (step_y / n))
    #                 chessboard_limit_points[2, l_counter] = 0
    #                 chessboard_limit_points[3, l_counter] = 1
    #                 l_counter += 1
    #
    #             if idx_y == 0:
    #                 for i in range(n, 0, -1):
    #                     chessboard_limit_points[0, l_counter] = x - step_x
    #                     chessboard_limit_points[1, l_counter] = y - ((n - i) * (step_y / n))
    #                     chessboard_limit_points[2, l_counter] = 0
    #                     chessboard_limit_points[3, l_counter] = 1
    #                     l_counter += 1
    #
    # dataset_chessboards['evaluation_points'] = chessboard_evaluation_points
    # # dataset_chessboards['limit_points'] = chessboard_limit_points
    # # dataset_chessboards['inner_points'] = chessboard_inner_points
    #
    # # print('chessboard_limit_points.shape=' + str(chessboard_limit_points.shape))
    # # exit(0)
    #
    # objp = np.zeros((nx * ny, 3), np.float32)
    # objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    # chessboard_points = np.transpose(objp)
    # chessboard_points = np.vstack(
    #     (chessboard_points, np.ones((1, nx * ny), dtype=np.float)))
    #
    # pts_l_chess = np.zeros((3, l_counter), np.float32)
    # for i in range(0, l_counter):
    #     pts_l_chess[0, i] = chessboard_limit_points[0, i]
    #     pts_l_chess[1, i] = chessboard_limit_points[1, i]
    #
    # pts_i_chess = np.zeros((3, i_counter), np.float32)
    # for i in range(0, i_counter):
    #     pts_i_chess[0, i] = chessboard_inner_points[0, i]
    #     pts_i_chess[1, i] = chessboard_inner_points[1, i]
    #
    # # homogenize points
    # pts_l_chess = np.vstack((pts_l_chess, np.ones((1, pts_l_chess.shape[1]), dtype=np.float)))
    #
    # dataset_chessboard_points = {'points': chessboard_points, 'l_points': pts_l_chess, 'i_points': pts_i_chess}
    #
    # for collection_key, collection in dataset['collections'].items():
    #     flg_detected_pattern = False
    #     for sensor_key, sensor in dataset['sensors'].items():
    #
    #         if not collection['labels'][sensor_key]['detected']:  # if chessboard not detected by sensor in collection
    #             print('Collection ' + str(collection_key) + ': Chessboard not detected by sensor ' + str(sensor_key))
    #             continue
    #
    #         # change accordingly to the first camera to give chessboard first poses
    #         if (sensor['msg_type'] == 'Image'):
    #
    #             K = np.ndarray((3, 3), dtype=np.float, buffer=np.array(sensor['camera_info']['K']))
    #             D = np.ndarray((5, 1), dtype=np.float, buffer=np.array(sensor['camera_info']['D']))
    #
    #             # TODO should we not read these from the dictionary?
    #             objp = np.zeros((nx * ny, 3), np.float32)
    #             objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    #             # Build a numpy array with the chessboard corners
    #             corners = np.zeros((len(collection['labels'][sensor_key]['idxs']), 1, 2), dtype=np.float)
    #             ids = range(0, len(collection['labels'][sensor_key]['idxs']))
    #             for idx, point in enumerate(collection['labels'][sensor_key]['idxs']):
    #                 corners[idx, 0, 0] = point['x']
    #                 corners[idx, 0, 1] = point['y']
    #                 ids[idx] = point['id']
    #
    #             # Find pose of the camera w.r.t the chessboard
    #             print(np.shape(objp))
    #             print(np.shape(corners))
    #             # projected, _, _ = utilities.projectToCamera(K, D, width, height, np.dot(sTc, pattern['grid'].T[ids].T))
    #             # ret, rvecs, tvecs = cv2.solvePnP(objp, corners, K, D)
    #             ret, rvecs, tvecs = cv2.solvePnP(objp[ids], np.array(corners, dtype=np.float32), K, D)
    #
    #             # Compute the pose of he chessboard w.r.t the base_link
    #             root_T_sensor = utilities.getAggregateTransform(sensor['chain'], collection['transforms'])
    #             sensor_T_chessboard = utilities.traslationRodriguesToTransform(tvecs, rvecs)
    #             root_T_chessboard = np.dot(root_T_sensor, sensor_T_chessboard)
    #             T = deepcopy(root_T_chessboard)
    #             T[0:3, 3] = 0  # remove translation component from 4x4 matrix
    #
    #             print('Creating first guess for collection ' + collection_key + ' using sensor ' + sensor_key)
    #             dataset_chessboards['collections'][str(collection_key)] = {
    #                 'trans': list(root_T_chessboard[0:3, 3]),
    #                 'quat': list(transformations.quaternion_from_matrix(T))}
    #
    #             flg_detected_pattern = True
    #             break  # don't search for this collection's chessboard on anymore sensors
    #
    #     if not flg_detected_pattern:  # Abort when the chessboard is not detected by any camera on this collection
    #         raise ValueError('Collection ' + collection_key + ' could not find chessboard.')
    #
    # return dataset_chessboards, dataset_chessboard_points
