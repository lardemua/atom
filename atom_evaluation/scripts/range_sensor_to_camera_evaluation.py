#!/usr/bin/env python

"""
Reads the calibration results from a json file and computes the evaluation metrics
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import json
import os
import numpy as np
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)

    args = vars(ap.parse_args())

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    # ---------------------------------------
    # --- Get calibrated transformation
    # ---------------------------------------
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    from_frame = dataset['calibration_config']['sensors'][target_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][source_sensor]['link']
    for collection_key, collection in dataset['collections'].items():
        # -- Get velodyne to camera transformation
        vel2cam = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])

        # -- Convert limit points from velodyne to camera frame
        pts = collection['labels'][source_sensor]['limit_points']
        points_in_vel = np.array(
            [[item['x'] for item in pts], [item['y'] for item in pts], [item['z'] for item in pts],
             [1 for item in pts]], np.float)

        points_in_cam = np.dot(vel2cam, points_in_vel)

        # -- Project them to the image
        w, h = collection['data'][target_sensor]['width'], collection['data'][target_sensor]['height']
        K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][target_sensor]['camera_info']['K']), dtype=np.float)
        D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][target_sensor]['camera_info']['D']), dtype=np.float)

        pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

        filename = os.path.dirname(args['json_file']) + '/' + collection['data'][target_sensor]['data_file']
        image = cv2.imread(filename)

        for idx in range(0, pts_in_image.shape[1]):
            image = cv2.circle(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])), 5, (255, 0, 0), -1)
        cv2.imshow("LiDAR Reprojection", image)
        cv2.waitKey()
