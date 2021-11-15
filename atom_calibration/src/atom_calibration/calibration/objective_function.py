# stdlib
import atom_core.atom
import math
from copy import deepcopy

import chardet
import numpy as np
import ros_numpy
from scipy.spatial import distance

# 3rd-party
import OptimizationUtils.utilities as opt_utilities
from geometry_msgs.msg import Point

# Own modules
from atom_core.dataset_io import getPointCloudMessageFromDictionary
from atom_core.geometry import distance_two_3D_points, isect_line_plane_v3
from atom_core.cache import Cache


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

@Cache(args_to_ignore=['_dataset'])
def getPointsInPatternAsNPArray(_collection_key, _sensor_key, _dataset):
    pts_in_pattern_list = []  # collect the points
    for pt_detected in _dataset['collections'][_collection_key]['labels'][_sensor_key]['idxs']:
        id_detected = pt_detected['id']
        point = [item for item in _dataset['patterns']['corners'] if item['id'] == id_detected][0]
        pts_in_pattern_list.append(point)

    return np.array([[item['x'] for item in pts_in_pattern_list],  # convert list to np array
                     [item['y'] for item in pts_in_pattern_list],
                     [0 for _ in pts_in_pattern_list],
                     [1 for _ in pts_in_pattern_list]], np.float)


@Cache(args_to_ignore=['_dataset'])
def getPointsDetectedInImageAsNPArray(_collection_key, _sensor_key, _dataset):
    return np.array(
        [[item['x'] for item in _dataset['collections'][_collection_key]['labels'][_sensor_key]['idxs']],
         [item['y'] for item in _dataset['collections'][_collection_key]['labels'][_sensor_key]['idxs']]],
        dtype=np.float)


@Cache(args_to_ignore=['_dataset'])
def getPointsInSensorAsNPArray(_collection_key, _sensor_key, _label_key, _dataset):
    cloud_msg = getPointCloudMessageFromDictionary(_dataset['collections'][_collection_key]['data'][_sensor_key])
    idxs = _dataset['collections'][_collection_key]['labels'][_sensor_key][_label_key]
    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points = np.zeros((4, pc.shape[0]))
    points[0, :] = pc['x']
    points[1, :] = pc['y']
    points[2, :] = pc['z']
    points[3, :] = 1
    return points


# @Cache(args_to_ignore=['residuals', 'dataset'])
# def getNormalizerForMsgType(msg_type, residuals, dataset):
#     values = []
#     for sensor_key, sensor in dataset['sensors'].items():
#         if sensor['msg_type'] == msg_type:
#             values += [residuals[k] for k in residuals.keys() if sensor_key in k]
#
#     return np.mean(values)

@Cache(args_to_ignore=['residuals', 'dataset'])
def getNormalizerForMsgType(sensor_id, residuals, dataset):
    values = []
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['sensor_id'] == sensor_id:
            values += [residuals[k] for k in residuals.keys() if sensor_key in k]

    return np.mean(values)


@Cache(args_to_ignore=['keys'])
def getResKeysForSensor(sensor_key, keys):
    return [k for k in keys if sensor_key in k]


def objectiveFunction(data):
    """
    Computes a list or a dictionary of residuals. There should be an error for each stamp, sensor and chessboard tuple.
    The computation of the error varies according with the modality of the sensor:
        - Reprojection error for camera to chessboard
        - Point to plane distance for 2D laser scanners
        - (...)
        :return: a vector of residuals
    """
    # print('Calling objective function.')

    # Get the data from the model
    dataset = data['dataset']
    patterns = data['dataset']['patterns']
    args = data['args']
    if args['view_optimization'] or args['ros_visualization']:
        dataset_graphics = data['graphics']

    normalizer = data['normalizer']



    r = {}  # Initialize residuals dictionary.
    for collection_key, collection in dataset['collections'].items():

        # from tf import transformations
        # print('collection ' + str(collection_key))
        # m = atom_core.atom.getTransform('world', 'hand_camera_link', collection['transforms'])
        # q = transformations.quaternion_from_matrix(m)
        # print('world to hand_camera_link = ' + str(m) + '\nquat = ' + str(q))
        #
        # m = atom_core.atom.getTransform('world', 'world_camera_link', collection['transforms'])
        # q = transformations.quaternion_from_matrix(m)
        # print('world to world_camera_link = ' + str(m) + '\nquat = ' + str(q))

        for sensor_key, sensor in dataset['sensors'].items():

            if not collection['labels'][sensor_key]['detected']:  # chess not detected by sensor in collection
                continue

            # if sensor['msg_type'] == 'Image':
            if sensor['sensor_id'] == 'rgb':
                # Get the pattern corners in the local pattern frame. Must use only corners which have -----------------
                # correspondence to the detected points stored in collection['labels'][sensor_key]['idxs'] -------------
                pts_in_pattern = getPointsInPatternAsNPArray(collection_key, sensor_key, dataset)



                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = sensor['parent']
                to_frame = dataset['calibration_config']['calibration_pattern']['link']
                sensor_to_pattern = atom_core.atom.getTransform(from_frame, to_frame, collection['transforms'])
                pts_in_sensor = np.dot(sensor_to_pattern, pts_in_pattern)

                # q = transformations.quaternion_from_matrix(sensor_to_pattern)
                # print('T =\n' + str(sensor_to_pattern) + '\nquat = ' + str(q))

                # Project points to the image of the sensor ------------------------------------------------------------
                w, h = collection['data'][sensor_key]['width'], collection['data'][sensor_key]['height']
                K = np.ndarray((3, 3), buffer=np.array(sensor['camera_info']['K']), dtype=np.float)
                D = np.ndarray((5, 1), buffer=np.array(sensor['camera_info']['D']), dtype=np.float)

                pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, pts_in_sensor[0:3, :])

                # Get the detected points to use as ground truth--------------------------------------------------------
                pts_detected_in_image = getPointsDetectedInImageAsNPArray(collection_key, sensor_key, dataset)

                # Compute the residuals as the distance between the pt_in_image and the pt_detected_in_image
                for idx, label_idx in enumerate(collection['labels'][sensor_key]['idxs']):
                    rname = 'c' + str(collection_key) + '_' + str(sensor_key) + '_corner' + str(label_idx['id'])
                    r[rname] = np.sqrt((pts_in_image[0, idx] - pts_detected_in_image[0, idx]) ** 2 +
                                       (pts_in_image[1, idx] - pts_detected_in_image[1, idx]) ** 2) / normalizer[
                                   'rgb']

                # Required by the visualization function to publish annotated images
                idxs_projected = []
                for idx in range(0, pts_in_image.shape[1]):
                    idxs_projected.append({'x': pts_in_image[0][idx], 'y': pts_in_image[1][idx]})
                collection['labels'][sensor_key]['idxs_projected'] = idxs_projected  # store projections

                if 'idxs_initial' not in collection['labels'][sensor_key]:  # store the first projections
                    collection['labels'][sensor_key]['idxs_initial'] = deepcopy(idxs_projected)



            # elif sensor['msg_type'] == 'LaserScan':
            elif sensor['sensor_id']=='laserscan':
                # Get laser points that belong to the chessboard
                idxs = collection['labels'][sensor_key]['idxs']
                rhos = [collection['data'][sensor_key]['ranges'][idx] for idx in idxs]
                thetas = [collection['data'][sensor_key]['angle_min'] +
                          collection['data'][sensor_key]['angle_increment'] * idx for idx in idxs]

                # Convert from polar to cartesian coordinates and create np array with xyz coords
                # TODO could be done only once
                pts_in_laser = np.zeros((4, len(rhos)), np.float32)
                for idx, (rho, theta) in enumerate(zip(rhos, thetas)):
                    pts_in_laser[0, idx] = rho * math.cos(theta)
                    pts_in_laser[1, idx] = rho * math.sin(theta)
                    pts_in_laser[2, idx] = 0
                    pts_in_laser[3, idx] = 1

                from_frame = dataset['calibration_config']['calibration_pattern']['link']
                to_frame = sensor['parent']
                pattern_to_sensor = atom_core.atom.getTransform(from_frame, to_frame, collection['transforms'])
                pts_in_chessboard = np.dot(pattern_to_sensor, pts_in_laser)

                # Compute the coordinate of the laser points in the chessboard reference frame
                # root_to_sensor = opt_utilities.getAggregateTransform(sensor['chain'], collection['transforms'])
                # pts_in_root = np.dot(root_to_sensor, pts_in_laser)
                #
                # transform_key = opt_utilities.generateKey(sensor['calibration_parent'], sensor['calibration_child'])
                # trans = collection['transforms'][transform_key]['trans']
                # quat = collection['transforms'][transform_key]['quat']
                #
                #
                #
                # chessboard_to_root = np.linalg.inv(opt_utilities.translationQuaternionToTransform(trans, quat))
                # pts_in_chessboard = np.dot(chessboard_to_root, pts_in_root)

                # --- Residuals: longitudinal error for extrema
                pts = []
                pts.extend(patterns['frame']['lines_sampled']['left'])
                pts.extend(patterns['frame']['lines_sampled']['right'])
                pts.extend(patterns['frame']['lines_sampled']['top'])
                pts.extend(patterns['frame']['lines_sampled']['bottom'])
                pts_canvas_in_chessboard = np.array([[pt['x'] for pt in pts], [pt['y'] for pt in pts]], np.float)

                # pts_canvas_in_chessboard = patterns['limit_points'][0:2, :].transpose()

                # compute minimum distance to inner_pts for right most edge (first in pts_in_chessboard list)
                extrema_right = np.reshape(pts_in_chessboard[0:2, 0], (2, 1))  # longitudinal -> ignore z values
                rname = collection_key + '_' + sensor_key + '_eright'
                r[rname] = float(np.amin(distance.cdist(extrema_right.transpose(),
                                                        pts_canvas_in_chessboard.transpose(), 'euclidean'))) / \
                           normalizer['laserscan']

                # compute minimum distance to inner_pts for left most edge (last in pts_in_chessboard list)
                extrema_left = np.reshape(pts_in_chessboard[0:2, -1], (2, 1))  # longitudinal -> ignore z values
                rname = collection_key + '_' + sensor_key + '_eleft'
                r[rname] = float(np.amin(distance.cdist(extrema_left.transpose(),
                                                        pts_canvas_in_chessboard.transpose(), 'euclidean'))) / \
                           normalizer['laserscan']

                # --- Residuals: Longitudinal distance for inner points
                pts = []
                pts.extend(patterns['frame']['lines_sampled']['left'])
                pts.extend(patterns['frame']['lines_sampled']['right'])
                pts.extend(patterns['frame']['lines_sampled']['top'])
                pts.extend(patterns['frame']['lines_sampled']['bottom'])
                pts_inner_in_chessboard = np.array([[pt['x'] for pt in pts], [pt['y'] for pt in pts]], np.float)
                # pts_inner_in_chessboard = patterns['inner_points'][0:2, :].transpose()
                edges2d_in_chessboard = pts_in_chessboard[0:2, collection['labels'][sensor_key]['edge_idxs']]  # this
                # is a longitudinal residual, so ignore z values.

                for idx in xrange(
                        edges2d_in_chessboard.shape[1]):  # compute minimum distance to inner_pts for each edge
                    xa = np.reshape(edges2d_in_chessboard[:, idx], (2, 1)).transpose()  # need the reshape because this
                    # becomes a shape (2,) which the function cdist does not support.

                    rname = collection_key + '_' + sensor_key + '_inner_' + str(idx)
                    r[rname] = float(np.amin(distance.cdist(xa, pts_inner_in_chessboard.transpose(), 'euclidean'))) / \
                               normalizer['laserscan']

                # --- Residuals: Beam direction distance from point to chessboard plan
                # For computing the intersection we need:
                # p0, p1: Define the line.
                # p_co, p_no: define the plane:
                # p_co Is a point on the plane (plane coordinate).
                # p_no Is a normal vector defining the plane direction (does not need to be normalized).

                # Compute p0 and p1: p1 will be all the lidar data points, i.e., pts_in_laser, p0 will be the origin
                # of the laser sensor. Compute the p0_in_laser (p0)
                p0_in_laser = np.array([[0], [0], [0], [1]], np.float)

                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = sensor['parent']
                to_frame = dataset['calibration_config']['calibration_pattern']['link']
                laser_to_chessboard = atom_core.atom.getTransform(from_frame, to_frame, collection['transforms'])

                p_co_in_chessboard = np.array([[0], [0], [0], [1]], np.float)
                p_co_in_laser = np.dot(laser_to_chessboard, p_co_in_chessboard)

                # Compute p_no. First compute an aux point (p_caux) and then use the vector from p_co to p_caux.
                p_caux_in_chessboard = np.array([[0], [0], [1], [1]], np.float)  # along the zz axis (plane normal)
                p_caux_in_laser = np.dot(laser_to_chessboard, p_caux_in_chessboard)

                p_no_in_laser = np.array([[p_caux_in_laser[0] - p_co_in_laser[0]],
                                          [p_caux_in_laser[1] - p_co_in_laser[1]],
                                          [p_caux_in_laser[2] - p_co_in_laser[2]],
                                          [1]], np.float)  # plane normal

                if args['ros_visualization']:
                    marker = [x for x in dataset_graphics['ros']['MarkersLaserBeams'].markers if
                              x.ns == str(collection_key) + '-' + str(sensor_key)][0]
                    marker.points = []
                    rviz_p0_in_laser = Point(p0_in_laser[0], p0_in_laser[1], p0_in_laser[2])

                for idx in xrange(0, pts_in_laser.shape[1]):  # for all points
                    rho = rhos[idx]
                    p1_in_laser = pts_in_laser[:, idx]
                    pt_intersection = isect_line_plane_v3(p0_in_laser, p1_in_laser, p_co_in_laser, p_no_in_laser)

                    if pt_intersection is None:
                        raise ValueError('Pattern is almost parallel to the laser beam! Please remove collection ' +
                                         collection_key)

                    rname = collection_key + '_' + sensor_key + '_beam_' + str(idx)
                    r[rname] = abs(distance_two_3D_points(p0_in_laser, pt_intersection) - rho) / normalizer['laserscan']

                    if args['ros_visualization']:
                        marker.points.append(deepcopy(rviz_p0_in_laser))
                        marker.points.append(Point(pt_intersection[0], pt_intersection[1], pt_intersection[2]))

            # elif sensor['msg_type'] == 'PointCloud2':
            elif sensor['sensor_id'] == 'lidar':
                # Get the 3D LiDAR labelled points for the given collection
                points_in_sensor = getPointsInSensorAsNPArray(collection_key, sensor_key, 'idxs', dataset)

                # ------------------------------------------------------------------------------------------------
                # --- Orthogonal Distance Residuals: Distance from 3D range sensor point to chessboard plan
                # ------------------------------------------------------------------------------------------------
                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = dataset['calibration_config']['calibration_pattern']['link']
                to_frame = sensor['parent']
                lidar_to_pattern = atom_core.atom.getTransform(from_frame, to_frame, collection['transforms'])

                # TODO we could also use the middle points ...
                # points_in_pattern = np.dot(lidar_to_pattern, detected_middle_points_in_sensor)
                points_in_pattern = np.dot(lidar_to_pattern, points_in_sensor)

                rname_pre = 'c' + collection_key + '_' + sensor_key + '_oe_'
                for idx in collection['labels'][sensor_key]['samples']:
                    # Compute the residual: absolute of z component
                    rname = rname_pre + str(idx)
                    r[rname] = float(abs(points_in_pattern[2, idx])) / normalizer['lidar']

                # ------------------------------------------------------------------------------------------------
                # --- Pattern Extrema Residuals: Distance from the extremas of the pattern to the extremas of the cloud
                # ------------------------------------------------------------------------------------------------
                detected_limit_points_in_sensor = getPointsInSensorAsNPArray(collection_key, sensor_key,
                                                                              'idxs_limit_points', dataset)

                from_frame = dataset['calibration_config']['calibration_pattern']['link']
                to_frame = sensor['parent']
                pattern_to_sensor = atom_core.atom.getTransform(from_frame, to_frame, collection['transforms'])
                detected_limit_points_in_pattern = np.dot(pattern_to_sensor, detected_limit_points_in_sensor)

                pts = []
                pts.extend(patterns['frame']['lines_sampled']['left'])
                pts.extend(patterns['frame']['lines_sampled']['right'])
                pts.extend(patterns['frame']['lines_sampled']['top'])
                pts.extend(patterns['frame']['lines_sampled']['bottom'])
                ground_truth_limit_points_in_pattern = np.array([[pt['x'] for pt in pts], [pt['y'] for pt in pts]],
                                                                np.float)

                # Compute and save residuals
                for idx in range(detected_limit_points_in_pattern.shape[1]):
                    m_pt = np.reshape(detected_limit_points_in_pattern[0:2, idx], (1, 2))
                    rname = 'c' + collection_key + '_' + sensor_key + '_ld_' + str(idx)
                    r[rname] = np.min(distance.cdist(m_pt,
                                                     ground_truth_limit_points_in_pattern.transpose(), 'euclidean')) / \
                               normalizer['lidar']
                # ------------------------------------------------------------------------------------------------

            else:
                raise ValueError("Unknown sensor msg_type or sensor_id")

    if args['verbose']:
        print("Errors per sensor:")
        for sensor_key, sensor in dataset['sensors'].items():
            keys = [k for k in r.keys() if sensor_key in k]
            v = [r[k] * normalizer[sensor['sensor_id']] for k in keys]
            print('  ' + sensor_key + " " + str(np.mean(v)))

        for collection_key, collection in dataset['collections'].items():
            v = []
            for sensor_key, sensor in dataset['sensors'].items():
                keys = [k for k in r.keys() if ('c' + collection_key) == k.split('_')[0] and sensor_key in k]
                v = [r[k] * normalizer[sensor['sensor_id']] for k in keys]
                print('Collection ' + collection_key + ' ' + sensor_key + ' has ' + str(np.mean(v)))

        # per_col_sensor = {str(c): {str(s): {'avg': mean([r[k] for k in r.keys() if c == k.split('_')[0] and s in k]),
        #                                     'navg': mean([rn[k] for k in rn.keys() if c == k.split('_')[0] and s in k])}
        #                            for s in dataset['sensors']} for c in dataset['collections']}

    return r  # Return the residuals
