#!/usr/bin/env python
"""
A set of utilities to be used in the optimization algorithms
"""

# -------------------------------------------------------------------------------
# --- IMPORTS (standard, then third party, then my own modules)
# -------------------------------------------------------------------------------


# Standard imports
import os

import cv2
import atom_core.ros_numpy
import open3d as o3d
import numpy as np
from numpy.linalg import norm

# ROS imports
from image_geometry import PinholeCameraModel
from rospy_message_converter import message_converter

# Atom imports
from atom_calibration.collect.label_messages import convertDepthImage16UC1to32FC1
from atom_core.dataset_io import  getPointCloudMessageFromDictionary, read_pcd

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def projectToCamera(intrinsic_matrix, distortion, width, height, pts):
    """
    Projects a list of points to the camera defined transform, intrinsics and distortion
    :param intrinsic_matrix: 3x3 intrinsic camera matrix
    :param distortion: should be as follows: (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]])
    :param width: the image width
    :param height: the image height
    :param pts: a list of point coordinates (in the camera frame) with the following format: np array 4xn or 3xn
    :return: a list of pixel coordinates with the same length as pts
    """

    # print('intrinsic_matrix=' + str(intrinsic_matrix))
    # print('distortion=' + str(distortion))
    # print('width=' + str(width))
    # print('height=' + str(height))
    # print('pts.shape=' + str(pts.shape))
    _, n_pts = pts.shape

    # Project the 3D points in the camera's frame to image pixels
    # From https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    pixs = np.zeros((2, n_pts), dtype=float)

    k1, k2, p1, p2, k3 = distortion
    # fx, _, cx, _, fy, cy, _, _, _ = intrinsic_matrix
    # print('intrinsic=\n' + str(intrinsic_matrix))
    fx = intrinsic_matrix[0, 0]
    fy = intrinsic_matrix[1, 1]
    cx = intrinsic_matrix[0, 2]
    cy = intrinsic_matrix[1, 2]

    x = pts[0, :]
    y = pts[1, :]
    z = pts[2, :]

    dists = norm(pts[0:3, :], axis=0)  # compute distances from point to camera
    xl = np.divide(x, z)  # compute homogeneous coordinates
    yl = np.divide(y, z)  # compute homogeneous coordinates
    r2 = xl ** 2 + yl ** 2  # r square (used multiple times bellow)
    xll = xl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + 2 * p1 * xl * yl + p2 * (r2 + 2 * xl ** 2)
    yll = yl * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) + p1 * (r2 + 2 * yl ** 2) + 2 * p2 * xl * yl
    pixs[0, :] = fx * xll + cx
    pixs[1, :] = fy * yll + cy

    # Compute mask of valid projections
    valid_z = z > 0
    valid_xpix = np.logical_and(pixs[0, :] >= 0, pixs[0, :] < width)
    valid_ypix = np.logical_and(pixs[1, :] >= 0, pixs[1, :] < height)
    valid_pixs = np.logical_and(valid_z, np.logical_and(valid_xpix, valid_ypix))
    return pixs, valid_pixs, dists


def projectWithoutDistortion(intrinsic_matrix, width, height, pts):
    """
    Projects a list of points to the camera defined transform, intrinsics and distortion
    :param intrinsic_matrix: 3x3 intrinsic camera matrix
    :param width: the image width
    :param height: the image height
    :param pts: a list of point coordinates (in the camera frame) with the following format
    :return: a list of pixel coordinates with the same length as pts
    """

    _, n_pts = pts.shape

    # Project the 3D points in the camera's frame to image pixels without considering the distorcion
    # From https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    pixs = np.zeros((2, n_pts), dtype=float)

    # fx, _, cx, _, fy, cy, _, _, _ = intrinsic_matrix

    fx = intrinsic_matrix[0, 0]
    fy = intrinsic_matrix[1, 1]
    cx = intrinsic_matrix[0, 2]
    cy = intrinsic_matrix[1, 2]

    x = pts[0, :]
    y = pts[1, :]
    z = pts[2, :]

    dists = norm(pts[0:3, :], axis=0)  # compute distances from point to camera
    xl = np.divide(x, z)  # compute homogeneous coordinates
    yl = np.divide(y, z)  # compute homogeneous coordinates

    pixs[0, :] = fx * xl + cx
    pixs[1, :] = fy * yl + cy

    # Compute mask of valid projections
    valid_z = z > 0
    valid_xpix = np.logical_and(pixs[0, :] >= 0, pixs[0, :] < width)
    valid_ypix = np.logical_and(pixs[1, :] >= 0, pixs[1, :] < height)
    valid_pixs = np.logical_and(valid_z, np.logical_and(valid_xpix, valid_ypix))
    return pixs, valid_pixs, dists

def convert_from_uvd(cx, cy, fx, fy, xpix, ypix, d):
    # From http://www.open3d.org/docs/0.7.0/python_api/open3d.geometry.create_point_cloud_from_depth_image.html

    x_over_z = (xpix - cx) / fx
    y_over_z = (ypix - cy) / fy
    z = d
    x = x_over_z * z
    y = y_over_z * z
    return x, y, z

def depthToNPArray(dataset, selected_collection_key, json_file, ss, idxs):
    """
    Convert a depth image to numpy array
    """
    
    filename = os.path.dirname(json_file) + '/' + \
                dataset['collections'][selected_collection_key]['data'][ss]['data_file']

    cv_image_int16_tenths_of_millimeters = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    
    img = convertDepthImage16UC1to32FC1(cv_image_int16_tenths_of_millimeters,
                                        scale=10000.0)

    pinhole_camera_model = PinholeCameraModel()
    pinhole_camera_model.fromCameraInfo(message_converter.convert_dictionary_to_ros_message(
        'sensor_msgs/CameraInfo', dataset['sensors'][ss]['camera_info']))

    f_x = pinhole_camera_model.fx()
    f_y = pinhole_camera_model.fy()
    c_x = pinhole_camera_model.cx()
    c_y = pinhole_camera_model.cy()
    w = pinhole_camera_model.fullResolution()[0]

    # initialize lists
    xs = []
    ys = []
    zs = []
    for idx in idxs:  # iterate all points
        # convert from linear idx to x_pix and y_pix indices.
        y_pix = int(idx / w)
        x_pix = int(idx - y_pix * w)

        # get distance value for this pixel coordinate
        distance = img[y_pix, x_pix]

        if np.isnan(distance): # if the value of the pixel is nan, ignore it
            continue

        # compute 3D point and add to list of coordinates xs, ys, zs
        x, y, z = convert_from_uvd(c_x, c_y, f_x, f_y, x_pix, y_pix, distance)
        xs.append(x)
        ys.append(y)
        zs.append(z)

    homogeneous = np.ones((len(xs)))
    points_in_depth = np.array((xs, ys, zs, homogeneous), dtype=float)

    return points_in_depth


def depthToImage(dataset, selected_collection_key, json_file, pattern_key, ss, ts, tf):
    """
    Convert a depth image to points inside an rgb image
    """

    idxs = dataset['collections'][selected_collection_key]['labels'][pattern_key][ss]['idxs_limit_points']
    points_in_depth = depthToNPArray(dataset, selected_collection_key, json_file, ss, idxs)

    points_in_cam = np.dot(tf, points_in_depth)

    # -- Project them to the image
    w, h = dataset['collections'][selected_collection_key]['data'][ts]['width'],\
            dataset['collections'][selected_collection_key]['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=float)

    pts_in_image, _, _ = projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image


def depthToPointCloud(dataset, selected_collection_key, json_file, ss, idxs):
    """
    Converts a depth image to an open3d pointcloud
    """
    points_in_depth = depthToNPArray(dataset, selected_collection_key, json_file, ss, idxs)
    points_3 = np.transpose(np.delete(points_in_depth, 3, 0))
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_3)


    return point_cloud

def depthInImage(dataset, selected_collection_key, pattern_key, ss):
    """
    Converts depth idxs_limit_points to points inside the depth image
    """

    idxs = dataset['collections'][selected_collection_key]['labels'][pattern_key][ss]['idxs_limit_points']
    pinhole_camera_model = PinholeCameraModel()
    pinhole_camera_model.fromCameraInfo(message_converter.convert_dictionary_to_ros_message(
        'sensor_msgs/CameraInfo', dataset['sensors'][ss]['camera_info']))
    w = pinhole_camera_model.fullResolution()[0]

    # initialize lists
    xs = []
    ys = []
    for idx in idxs:  # iterate all points
        # convert from linear idx to x_pix and y_pix indices.
        y_pix = int(idx / w)
        x_pix = int(idx - y_pix * w)
        xs.append(x_pix)
        ys.append(y_pix)

    points_in_depth = np.array((xs, ys), dtype=float)
    return points_in_depth


def rangeToImage(mixed_dataset, collection, json_file, pattern_key, ss, ts, tf):
    filename = os.path.dirname(json_file) + '/' + collection['data'][ss]['data_file']
    msg = read_pcd(filename)
    collection['data'][ss].update(message_converter.convert_ros_message_to_dictionary(msg))

    cloud_msg = getPointCloudMessageFromDictionary(collection['data'][ss])
    idxs = collection['labels'][pattern_key][ss]['idxs_limit_points']

    pc = atom_core.ros_numpy.numpify(cloud_msg)[idxs]
    points_in_vel = np.zeros((4, pc.shape[0]))
    points_in_vel[0, :] = pc['x']
    points_in_vel[1, :] = pc['y']
    points_in_vel[2, :] = pc['z']
    points_in_vel[3, :] = 1

    points_in_cam = np.dot(tf, points_in_vel)

    # -- Project them to the image
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(mixed_dataset['sensors'][ts]['camera_info']['K']), dtype=float)
    D = np.ndarray((5, 1), buffer=np.array(mixed_dataset['sensors'][ts]['camera_info']['D']), dtype=float)

    lidar_pts_in_img, _, _ = projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return lidar_pts_in_img