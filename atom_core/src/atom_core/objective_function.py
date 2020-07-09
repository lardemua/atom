# stdlib
import pprint
import math
import time
from copy import deepcopy

# 3rd-party
import numpy as np

from statistics import mean

import rospy
import ros_numpy

from scipy.spatial import distance

from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from rospy_message_converter import message_converter
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray, Marker

import OptimizationUtils.utilities as opt_utilities


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def distance_two_3D_points(p0, p1):
    return math.sqrt(((p0[0] - p1[0]) ** 2) + ((p0[1] - p1[1]) ** 2) + ((p0[2] - p1[2]) ** 2))


# intersection function
def isect_line_plane_v3(p0, p1, p_co, p_no, epsilon=1e-6):
    """
    p0, p1: Define the line.
    p_co, p_no: define the plane:
        p_co Is a point on the plane (plane coordinate).
        p_no Is a normal vector defining the plane direction;
             (does not need to be normalized).

    Return a Vector or None (when the intersection can't be found).
    """

    u = sub_v3v3(p1, p0)
    dot = dot_v3v3(p_no, u)

    if abs(dot) > epsilon:
        # The factor of the point between p0 -> p1 (0 - 1)
        # if 'fac' is between (0 - 1) the point intersects with the segment.
        # Otherwise:
        #  < 0.0: behind p0.
        #  > 1.0: infront of p1.
        w = sub_v3v3(p0, p_co)
        fac = -dot_v3v3(p_no, w) / dot
        u = mul_v3_fl(u, fac)
        return add_v3v3(p0, u)
    else:
        # The segment is parallel to plane.
        return None


# ----------------------
# generic math functions


def add_v3v3(v0, v1):
    return (
        v0[0] + v1[0],
        v0[1] + v1[1],
        v0[2] + v1[2],
    )


def sub_v3v3(v0, v1):
    return (
        v0[0] - v1[0],
        v0[1] - v1[1],
        v0[2] - v1[2],
    )


def dot_v3v3(v0, v1):
    return (
            (v0[0] * v1[0]) +
            (v0[1] * v1[1]) +
            (v0[2] * v1[2])
    )


def len_squared_v3(v0):
    return dot_v3v3(v0, v0)


def mul_v3_fl(v0, f):
    return (
        v0[0] * f,
        v0[1] * f,
        v0[2] * f,
    )


def computeResidualsAverage(residuals):
    for _, r in residuals.items():
        if not r['count'] == 0:
            r['average'] = r['total'] / r['count']
        else:
            r['average'] = np.nan


def objectiveFunction(data):
    """
    Computes the vector of residuals. There should be an error for each stamp, sensor and chessboard tuple.
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
    # dataset_chessboard_points = data['dataset_chessboard_points']  # TODO should be integrated into chessboards
    args = data['args']
    if args['view_optimization'] or args['ros_visualization']:
        dataset_graphics = data['graphics']

    r = {}  # Initialize residuals dictionary.
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():

            if not collection['labels'][sensor_key]['detected']:  # chess not detected by sensor in collection
                continue

            if sensor['msg_type'] == 'Image':

                # Get the pattern corners in the local pattern frame. Must use only corners which have -----------------
                # correspondence to the detected points stored in collection['labels'][sensor_key]['idxs'] -------------
                pts_in_pattern_list = []  # Collect the points
                step = int(1 / float(args['sample_residuals']))
                for pt_detected in collection['labels'][sensor_key]['idxs'][::step]:
                    id_detected = pt_detected['id']
                    point = [item for item in patterns['corners'] if item['id'] == id_detected][0]
                    pts_in_pattern_list.append(point)

                pts_in_pattern = np.array([[item['x'] for item in pts_in_pattern_list],  # convert list to np array
                                           [item['y'] for item in pts_in_pattern_list],
                                           [0 for _ in pts_in_pattern_list],
                                           [1 for _ in pts_in_pattern_list]], np.float)

                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = sensor['parent']
                to_frame = dataset['calibration_config']['calibration_pattern']['link']
                sensor_to_pattern = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
                pts_in_sensor = np.dot(sensor_to_pattern, pts_in_pattern)

                # Project points to the image of the sensor ------------------------------------------------------------
                # TODO still not sure whether to use K or P (probably K) and distortion or not
                w, h = collection['data'][sensor_key]['width'], collection['data'][sensor_key]['height']
                K = np.ndarray((3, 3), buffer=np.array(sensor['camera_info']['K']), dtype=np.float)
                # P = np.ndarray((3, 4), buffer=np.array(sensor['camera_info']['P']), dtype=np.float)
                D = np.ndarray((5, 1), buffer=np.array(sensor['camera_info']['D']), dtype=np.float)

                pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, pts_in_sensor[0:3, :])
                # pts_in_image, _, _ = opt_utilities.projectToCamera(P, D, w, h, pts_in_sensor[0:3, :])
                # pts_in_image, _, _ = opt_utilities.projectWithoutDistortion(K, w, h, pts_in_sensor[0:3, :])
                # See issue #106
                # pts_in_image, _, _ = opt_utilities.projectWithoutDistortion(P, w, h, pts_in_sensor[0:3, :])

                # Get the detected points to use as ground truth--------------------------------------------------------
                pts_detected_in_image = np.array([[item['x'] for item in collection['labels'][sensor_key]['idxs'][::step]],
                                                  [item['y'] for item in collection['labels'][sensor_key]['idxs'][::step]]],
                                                 dtype=np.float)

                # Compute the residuals as the distance between the pt_in_image and the pt_detected_in_image
                # print(collection['labels'][sensor_key]['idxs'])
                for idx, label_idx in enumerate(collection['labels'][sensor_key]['idxs'][::step]):
                    rname = 'c' + str(collection_key) + '_' + str(sensor_key) + '_corner' + str(label_idx['id'])
                    r[rname] = math.sqrt((pts_in_image[0, idx] - pts_detected_in_image[0, idx]) ** 2 +
                                         (pts_in_image[1, idx] - pts_detected_in_image[1, idx]) ** 2)

                # Required by the visualization function to publish annotated images
                idxs_projected = []
                for idx in range(0, pts_in_image.shape[1]):
                    idxs_projected.append({'x': pts_in_image[0][idx], 'y': pts_in_image[1][idx]})
                collection['labels'][sensor_key]['idxs_projected'] = idxs_projected  # store projections

                if 'idxs_initial' not in collection['labels'][sensor_key]:  # store the first projections
                    collection['labels'][sensor_key]['idxs_initial'] = deepcopy(idxs_projected)

            elif sensor['msg_type'] == 'LaserScan':
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
                pattern_to_sensor = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
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
                r[rname] = float(
                    np.amin(
                        distance.cdist(extrema_right.transpose(), pts_canvas_in_chessboard.transpose(), 'euclidean')))

                # compute minimum distance to inner_pts for left most edge (last in pts_in_chessboard list)
                extrema_left = np.reshape(pts_in_chessboard[0:2, -1], (2, 1))  # longitudinal -> ignore z values
                rname = collection_key + '_' + sensor_key + '_eleft'
                r[rname] = float(
                    np.amin(
                        distance.cdist(extrema_left.transpose(), pts_canvas_in_chessboard.transpose(), 'euclidean')))

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

                for idx in range(edges2d_in_chessboard.shape[1]):  # compute minimum distance to inner_pts for each edge
                    xa = np.reshape(edges2d_in_chessboard[:, idx], (2, 1)).transpose()  # need the reshape because this
                    # becomes a shape (2,) which the function cdist does not support.

                    rname = collection_key + '_' + sensor_key + '_inner_' + str(idx)
                    r[rname] = float(np.amin(distance.cdist(xa, pts_inner_in_chessboard.transpose(), 'euclidean')))

                # --- Residuals: Beam direction distance from point to chessboard plan
                # For computing the intersection we need:
                # p0, p1: Define the line.
                # p_co, p_no: define the plane:
                # p_co Is a point on the plane (plane coordinate).
                # p_no Is a normal vector defining the plane direction (does not need to be normalized).

                # Compute p0 and p1: p1 will be all the lidar data points, i.e., pts_in_laser, p0 will be the origin
                # of the laser sensor. Compute the p0_in_laser (p0)
                p0_in_laser = np.array([[0], [0], [0], [1]], np.float)

                # Compute p_co. It can be any point in the chessboard plane. Lets transform the origin of the
                # chessboard to the laser reference frame
                # transform_key = opt_utilities.generateKey(sensor['calibration_parent'], sensor['calibration_child'])
                # trans = collection['transforms'][transform_key]['trans']
                # quat = collection['transforms'][transform_key]['quat']
                # root_to_pattern = opt_utilities.translationQuaternionToTransform(trans, quat)
                # laser_to_chessboard = np.dot(np.linalg.inv(root_to_sensor), root_to_pattern)

                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = sensor['parent']
                to_frame = dataset['calibration_config']['calibration_pattern']['link']
                laser_to_chessboard = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])

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

                for idx in range(0, pts_in_laser.shape[1]):  # for all points
                    rho = rhos[idx]
                    p1_in_laser = pts_in_laser[:, idx]
                    pt_intersection = isect_line_plane_v3(p0_in_laser, p1_in_laser, p_co_in_laser, p_no_in_laser)

                    if pt_intersection is None:
                        raise ValueError('Pattern is almost parallel to the laser beam! Please remove collection ' +
                                         collection_key)

                    rname = collection_key + '_' + sensor_key + '_beam_' + str(idx)
                    r[rname] = abs(distance_two_3D_points(p0_in_laser, pt_intersection) - rho)

                    if args['ros_visualization']:
                        marker.points.append(deepcopy(rviz_p0_in_laser))
                        marker.points.append(Point(pt_intersection[0], pt_intersection[1], pt_intersection[2]))

            elif sensor['msg_type'] == 'PointCloud2':

                # Get the 3D LiDAR labelled points for the given collection
                # pts = collection['labels'][sensor_key]['middle_points']
                # detected_middle_points_in_sensor = np.array(
                #     [[pt[0] for pt in pts], [pt[1] for pt in pts], [pt[2] for pt in pts], [pt[3] for pt in pts]],
                #     np.float)

                points_in_sensor = collection['labels'][sensor_key]['labelled_points']

                # ------------------------------------------------------------------------------------------------
                # --- Orthogonal Distance Residuals: Distance from 3D range sensor point to chessboard plan
                # ------------------------------------------------------------------------------------------------
                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                from_frame = dataset['calibration_config']['calibration_pattern']['link']
                to_frame = sensor['parent']
                lidar_to_pattern = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])

                # points_in_pattern = np.dot(lidar_to_pattern, detected_middle_points_in_sensor)
                points_in_pattern = np.dot(lidar_to_pattern, points_in_sensor.transpose())

                step = int(1 / float(args['sample_residuals']))
                for idx in range(0, points_in_pattern.shape[1], step):
                    # Compute the residual: absolute of z component
                    rname = collection_key + '_' + sensor_key + '_oe_' + str(idx)
                    r[rname] = abs(points_in_pattern[2, idx])

                # ------------------------------------------------------------------------------------------------
                # --- Beam Distance Residuals: Distance from 3D range sensor point to chessboard plan
                # ------------------------------------------------------------------------------------------------
                # For computing the intersection we need:
                # p0, p1: Define the line.
                # p_co, p_no: define the plane:
                # p_co Is a point on the plane (plane coordinate).
                # p_no Is a normal vector defining the plane direction (does not need to be normalized).

                # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
                # from_frame = sensor['parent']
                # to_frame = dataset['calibration_config']['calibration_pattern']['link']
                # lidar_to_pattern = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
                #
                # # Origin of the chessboard (0, 0, 0, 1) homogenized to the 3D range sensor reference frame
                # p_co_in_chessboard = np.array([[0], [0], [0], [1]], np.float)
                # p_co_in_lidar = np.dot(lidar_to_pattern, p_co_in_chessboard)
                #
                # # Compute p_no. First compute an aux point (p_caux) and then use the normal vector from p_co to p_caux.
                # p_caux_in_chessboard = np.array([[0], [0], [1], [1]], np.float)  # along the zz axis (plane normal)
                # p_caux_in_lidar = np.dot(lidar_to_pattern, p_caux_in_chessboard)
                #
                # p_no_in_lidar = np.array([[p_caux_in_lidar[0] - p_co_in_lidar[0]],
                #                           [p_caux_in_lidar[1] - p_co_in_lidar[1]],
                #                           [p_caux_in_lidar[2] - p_co_in_lidar[2]],
                #                           [1]], np.float)  # plane normal
                #
                # if args['ros_visualization']:
                #     marker = [x for x in dataset_graphics['ros']['MarkersLaserBeams'].markers if
                #               x.ns == str(collection_key) + '-' + str(sensor_key)][0]
                #     marker.points = []
                #     rviz_p0_in_lidar = Point(0, 0, 0)
                #
                # # Define p0 - the origin of 3D range sensor reference frame - to compute the line that intercepts the
                # # chessboard plane in the loop
                # p0_in_lidar = np.array([[0], [0], [0], [1], np.float])
                # # print('There are ' + str(len(points)))
                # for idx in range(0, len(points_in_sensor)):
                #     # Compute the interception between the chessboard plane into the 3D sensor reference frame, and the
                #     # line that goes through the sensor origin to the measured point in the chessboard plane
                #     p1_in_lidar = points_in_sensor[idx, :]
                #     pt_intersection = isect_line_plane_v3(p0_in_lidar, p1_in_lidar, p_co_in_lidar, p_no_in_lidar)
                #
                #     if pt_intersection is None:
                #         raise ValueError('Error: pattern is almost parallel to the lidar beam! Please delete the '
                #                          'collections in question.')
                #
                #     # Compute the residual: distance from sensor origin from the interception minus the actual range
                #     # measure
                #     rname = collection_key + '_' + sensor_key + '_oe_' + str(idx)
                #     rho = np.sqrt(p1_in_lidar[0] ** 2 + p1_in_lidar[1] ** 2 + p1_in_lidar[2] ** 2)
                #     r[rname] = abs(distance_two_3D_points(p0_in_lidar, pt_intersection) - rho)
                #
                #     if args['ros_visualization']:
                #         marker.points.append(deepcopy(rviz_p0_in_lidar))
                #         marker.points.append(Point(pt_intersection[0], pt_intersection[1], pt_intersection[2]))

                # ------------------------------------------------------------------------------------------------
                # --- Pattern Extrema Residuals: Distance from the extremas of the pattern to the extremas of the cloud
                # ------------------------------------------------------------------------------------------------

                pts = collection['labels'][sensor_key]['limit_points']
                detected_limit_points_in_sensor = np.array(
                    [[pt[0] for pt in pts], [pt[1] for pt in pts], [pt[2] for pt in pts], [pt[3] for pt in pts]],
                    np.float)

                from_frame = dataset['calibration_config']['calibration_pattern']['link']
                to_frame = sensor['parent']
                pattern_to_sensor = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
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
                    rname = collection_key + '_' + sensor_key + '_ld_' + str(idx)
                    r[rname] = np.min(
                        distance.cdist(m_pt, ground_truth_limit_points_in_pattern.transpose(), 'euclidean'))
                # ------------------------------------------------------------------------------------------------

            else:
                raise ValueError("Unknown sensor msg_type")

    # --- Normalization of residuals.
    # TODO put normalized residuals to the square
    rn = deepcopy(r)  # make a copy of the non normalized dictionary.

    # Message type normalization. Pixels and meters should be weighted based on an adhoc defined meter_to_pixel factor.
    meter_to_pixel_factor = 200  # trial and error, the best technique around :-)
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not collection['labels'][sensor_key]['detected']:  # pattern not detected by sensor in collection
                continue

            if sensor['msg_type'] == 'LaserScan' or sensor['msg_type'] == 'PointCloud2':
                pair_keys = [k for k in rn.keys() if collection_key == k.split('_')[0] and sensor_key in k]  #
                # compute the residual keys that belong to this collection-sensor pair.
                rn.update({k: rn[k] * meter_to_pixel_factor for k in pair_keys})  # update the normalized dictionary.

    # Intra collection-sensor pair normalization.
    # print(rn.keys())
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not collection['labels'][sensor_key]['detected']:  # chess not detected by sensor in collection
                continue


            pair_keys = [k for k in rn.keys() if ('c' + collection_key) == k.split('_')[0] and sensor_key in k]
            # print('For collection ' + str(collection_key) + ' sensor ' + sensor_key)
            # print('pair keys: ' + str(pair_keys))

            if sensor['msg_type'] == 'Image':  # Intra normalization: for cameras there is nothing to do, since all
                # measurements have the same importance. Inter normalization, divide by the number of pixels considered.
                # rn.update({k: rn[k] / len(pair_keys) for k in pair_keys})
                pass # nothing to do

            elif sensor['msg_type'] == 'LaserScan':  # Intra normalization: longitudinal measurements (extrema (.25)
                # and inner (.25)] and orthogonal measurements (beam (0.5)). Inter normalization, consider the number of
                # measurements.
                extrema_keys = [k for k in pair_keys if '_eleft' in k or '_eright' in k]
                rn.update({k: 0.25 / len(extrema_keys) * rn[k] for k in extrema_keys})

                inner_keys = [k for k in pair_keys if '_inner' in k]
                rn.update({k: 0.25 / len(inner_keys) * rn[k] for k in inner_keys})

                beam_keys = [k for k in pair_keys if '_beam' in k]
                rn.update({k: 0.5 / len(beam_keys) * rn[k] for k in beam_keys})

            elif sensor['msg_type'] == 'PointCloud2':  # Intra normalization: p
                orthogonal_keys = [k for k in pair_keys if '_oe' in k]
                limit_distance_keys = [k for k in pair_keys if '_ld' in k]
                total = len(orthogonal_keys) + len(limit_distance_keys)

                # rn.update({k: 0.5 / len(orthogonal_keys) * rn[k] for k in orthogonal_keys})
                rn.update({k: (1.0 - len(orthogonal_keys)/total) * rn[k] for k in orthogonal_keys})

                # rn.update({k: 0.5 / len(limit_distance_keys) * rn[k] for k in limit_distance_keys})
                rn.update({k: (1.0 - len(limit_distance_keys)/total) * rn[k] for k in limit_distance_keys})


    # print('r=\n')
    # print(r)
    #
    # print('\nrn=\n')
    # print(rn)
    # exit(0)

    # for collection_key, collection in dataset['collections'].items():
    #     for sensor_key, sensor in dataset['sensors'].items():
    #         pair_keys = [k for k in rn.keys() if collection_key == k.split('_')[0] and sensor_key in k]
    #         extrema_keys = [k for k in pair_keys if 'eleft' in k or 'eright' in k]
    #         inner_keys = [k for k in pair_keys if 'inner' in k]
    #         beam_keys = [k for k in pair_keys if 'beam' in k]
    #         print('Collection ' + collection_key + ' ' + sensor_key + ' has ' + str(len(pair_keys)) +
    #               ' residuals: ' + str(len(extrema_keys)) + ' extrema, ' + str(len(inner_keys)) + ' inner, ' +
    #               str(len(beam_keys)) + ' beam.')
    #
    # per_col_sensor = {str(c): {str(s): {'avg': mean([r[k] for k in r.keys() if c == k.split('_')[0] and s in k]),
    #                                     'navg': mean([rn[k] for k in rn.keys() if c == k.split('_')[0] and s in k])}
    #                            for s in dataset['sensors']} for c in dataset['collections']}
    #

    per_sensor = {}
    for sensor_key, sensor in dataset['sensors'].items():
        per_sensor[str(sensor_key)] = {'avg': mean([r[k] for k in r.keys() if sensor_key in k]),
                                       'navg': mean([rn[k] for k in rn.keys() if sensor_key in k])}

    # Get a list of all msg_types
    msg_types = []
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not sensor['msg_type'] in msg_types:
                msg_types.append(sensor['msg_type'])

        break  # one collection is enough to get msg_type

    per_msg_type = {}
    for msg_type in msg_types:
        total = 0
        ntotal = 0
        n = 0
        for sensor_key, sensor in dataset['sensors'].items():
            if sensor['msg_type'] == msg_type:
                values = [r[k] for k in r.keys() if sensor_key in k]
                total += sum(values)
                ntotal += sum([rn[k] for k in rn.keys() if sensor_key in k])
                n += len(values)

        per_msg_type[msg_type] = {'avg': total / n, 'navg': ntotal / n}

    # report = {'0-per_col_sensor': per_col_sensor, '1-per_sensor': per_sensor, '2-per_msg_type': per_msg_type}
    report = {'1-per_sensor': per_sensor, '2-per_msg_type': per_msg_type}

    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(report)

    # print(r)
    return rn  # Return the residuals
    # return r  # Return the residuals
