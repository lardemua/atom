#!/usr/bin/env python3

"""
Projects a point cloud into an image
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

# Standard imports
import json
import os
import argparse
from collections import OrderedDict

import cv2
import numpy as np

# ROS imports
import ros_numpy
from matplotlib import cm

# Atom imports
from atom_core.vision import projectToCamera
from atom_core.dataset_io import read_pcd
from atom_core.atom import getTransform

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def walk(node):
    for key, item in node.items():
        if isinstance(item, dict):
            walk(item)
        else:
            if isinstance(item, np.ndarray) and key == 'data':  # to avoid saving images in the json
                del node[key]

            elif isinstance(item, np.ndarray):
                node[key] = item.tolist()
            pass


def rangeToImage(collection, ss, ts, tf, pts):
    # -- Convert limit points from velodyne to camera frame
    points_in_lidar = np.array(
        [[item[0] for item in pts], [item[1] for item in pts], [item[2] for item in pts],
         [1 for item in pts]], float)

    points_in_cam = np.dot(tf, points_in_lidar)

    # -- Remove points in cam with z < 0
    points_in_cam_ = []
    for idx in range(0, points_in_cam.shape[1]):
        if points_in_cam[2, idx] > 0:
            points_in_cam_.append(points_in_cam[:, idx])

    points_in_cam = np.array(
        [[item[0] for item in points_in_cam_], [item[1] for item in points_in_cam_],
         [item[2] for item in points_in_cam_],
         [1 for item in points_in_cam_]], float)

    # -- Project them to the image
    selected_collection_key = list(dataset['collections'].keys())[0]
    w, h = collection['data'][ts]['width'], dataset['collections'][selected_collection_key]['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=float)

    pts_in_image, valid_pixs, _ = projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image, valid_pixs


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str,
                    required=True)
    ap.add_argument("-ls", "--lidar_sensor", help="LiDAR sensor name.", type=str, required=True)
    ap.add_argument("-cs", "--camera_sensor", help="Camera sensor name.", type=str, required=True)
    ap.add_argument("-md", "--max_distance", type=int, required=False, default=5.0,
                    help="Used for scaling colors of points projected in the image (in m)." +
                    "From max distance onward, everything will be the same color")

    # - Save args
    args = vars(ap.parse_args())
    lidar_sensor = args['lidar_sensor']
    camera_sensor = args['camera_sensor']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    pc2s = []
    pc2_publishers = []
    from_frame = dataset['calibration_config']['sensors'][camera_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][lidar_sensor]['link']
    od = OrderedDict(sorted(dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Read point cloud from file
        # ---------------------------------------
        pcd_filename = os.path.dirname(json_file) + '/' + collection['data'][lidar_sensor]['data_file']
        frame_id = str(collection['data'][lidar_sensor]['header']['frame_id'])

        # Read point cloud
        msg = read_pcd(pcd_filename, cloud_header=None)
        pc = ros_numpy.numpify(msg)
        points = np.zeros((pc.shape[0], 3))
        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']

        # ---------------------------------------
        # --- Read image from file
        # ---------------------------------------
        cam_filename = os.path.dirname(json_file) + '/' + collection['data'][camera_sensor]['data_file']
        image = cv2.imread(cam_filename)
        img_size = image.shape

        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        selected_collection_key = list(dataset['collections'].keys())[0]
        lidar2cam = getTransform(from_frame, to_frame,
                                 dataset['collections'][selected_collection_key]['transforms'])
        pts_in_image, valid_pixs = rangeToImage(collection, lidar_sensor, camera_sensor, lidar2cam, points)

        # Use a color scale up to six meters. Use milimeters to define the color
        cm_sensors = cm.rainbow(np.linspace(0, 1, int(args['max_distance'])*1000))

        for idx in range(0, pts_in_image.shape[1]):
            if valid_pixs[idx]:
                # in ros x is forward from the sensor. Convert to milimeters to retrieve color from colormap
                color_idx = min(int(args['max_distance'])*1000-1, int(round(points[idx, 0] * 1000)))
                color = (cm_sensors[color_idx, 0]*255, cm_sensors[color_idx, 1]*255, cm_sensors[color_idx, 2]*255)
                image = cv2.circle(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])), 2, color, -1)

        cv2.imshow("Lidar to Camera reprojection - collection " + str(collection_key), image)
        key = cv2.waitKey()
        if key == ord('q'):
            print('You pressed q ... aborting.')
            break
