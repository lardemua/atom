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
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore


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


# Save to json file
def createJSONFile(output_file, input):
    D = deepcopy(input)
    walk(D)

    print("Saving the json output file to " + str(output_file) + ", please wait, it could take a while ...")
    f = open(output_file, 'w')
    json.encoder.FLOAT_REPR = lambda f: ("%.6f" % f)  # to get only four decimal places on the json file
    print >> f, json.dumps(D, indent=2, sort_keys=True)
    f.close()
    print("Completed.")


def rangeToImage(collection, ss, ts, tf):
    # -- Convert limit points from velodyne to camera frame
    pts = collection['labels'][ss]['limit_points']
    points_in_vel = np.array(
        [[item['x'] for item in pts], [item['y'] for item in pts], [item['z'] for item in pts],
         [1 for item in pts]], np.float)

    points_in_cam = np.dot(tf, points_in_vel)

    # -- Project them to the image
    w, h = collection['data'][ts]['width'], collection['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)

    pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image


mouseX, mouseY = 0, 0


def click(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y


def annotateLimits(image):
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click)

    extremas = {}
    [extremas.setdefault(x, []) for x in range(4)]
    colors = [(125, 125, 125), (0, 255, 0), (0, 0, 255), (125, 0, 125)]
    annotating = True
    i = 0
    while i < 4:
        cv2.imshow('image', image)
        k = cv2.waitKey(20) & 0xFF
        if k == ord('c'):
            break
        elif k == ord('s'):
            image = cv2.circle(image, (mouseX, mouseY), 5, colors[i], -1)
            extremas[i].append([mouseX, mouseY])
        elif k == ord('p'):
            i += 1

    cv2.destroyWindow('image')
    return extremas


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)
    ap.add_argument("-ef", "--eval_file", help="Path to file to read and/or write the evalutation data.", type=str,
                    required=True)
    ap.add_argument("-ua", "--use_annotation", help="If true, the limit points will be manually annotated.",
                    action='store_true', default=False)

    # - Save args
    args = vars(ap.parse_args())
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']
    show_images = args['show_images']
    eval_file = args['eval_file']
    use_annotation = args['use_annotation']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    # ---------------------------------------
    # --- INITIALIZATION Read evaluation data from file ---> if desired <---
    # ---------------------------------------
    if use_annotation is False:
        # Loads a json file containing the evaluation data
        f = open(eval_file, 'r')
        eval_data = json.load(f)
    else:
        print(Fore.BLUE + "  Annotation tool intructions:")
        print(Fore.GREEN + "   - To add a point to a class: click + 's'")
        print(Fore.GREEN + "   - To change class: 'p'")
        print(Fore.GREEN + "   - To stop the annotation anytime: 'c'")
        print(Fore.GREEN + "   - It ends when you end annotating the fourth class (four times 'p')")
        print(Fore.WHITE)

    # Declare output dict to save the evaluation data if desired
    output_dict = {}
    output_dict['ground_truth_pts'] = {}

    from_frame = dataset['calibration_config']['sensors'][target_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][source_sensor]['link']
    for collection_key, collection in dataset['collections'].items():
        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        vel2cam = opt_utilities.getTransform(from_frame, to_frame, collection['transforms'])
        print(vel2cam)
        pts_in_image = rangeToImage(collection, source_sensor, target_sensor, vel2cam)

        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        filename = os.path.dirname(args['json_file']) + '/' + collection['data'][target_sensor]['data_file']
        image = cv2.imread(filename)
        if use_annotation is False:
            limits_on_image = eval_data['ground_truth_pts'][collection_key]
        else:
            limits_on_image = annotateLimits(image)

        output_dict['ground_truth_pts'][collection_key] = {}
        for i, pts in limits_on_image.items():
            pts = np.array(pts)
            if (pts.size == 0):
                continue

            x = pts[:, 0]
            y = pts[:, 1]
            coefficients = np.polyfit(x, y, 3)
            poly = np.poly1d(coefficients)
            new_x = np.linspace(x[0], x[-1])
            new_y = poly(new_x)

            if show_images == True:
                for idx in range(0, len(new_x)):
                    image = cv2.circle(image, (int(new_x[idx]), int(new_y[idx])), 3, (0, 0, 255), -1)

            output_dict['ground_truth_pts'][collection_key][i] = []
            for idx in range(0, len(new_x)):
                output_dict['ground_truth_pts'][collection_key][i].append([new_x[idx], new_y[idx]])

        # ---------------------------------------
        # --- Evaluation metrics - reprojection error
        # ---------------------------------------
        # -- For each reprojected limit point, find the closest ground truth point and compute the distance to it
        for idx in range(0, pts_in_image.shape[1]):
            target_pt = pts_in_image[:, idx]
            target_pt = np.reshape(target_pt[0:2], (2, 1))
            min_dist = 1e6
            for i, pts in output_dict['ground_truth_pts'][collection_key].items():
                dist = np.min(distance.cdist(target_pt.transpose(), pts, 'euclidean'))

                if dist < min_dist:
                    min_dist = dist
                    arg = np.argmin(distance.cdist(target_pt.transpose(), pts, 'euclidean'))
                    closest_pt = pts[arg]

            if show_images is True:
                image = cv2.line(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])),
                                 (int(closest_pt[0]), int(closest_pt[1])), (0, 255, 255), 3)

        # ---------------------------------------
        # --- Drawing ...
        # ---------------------------------------
        if show_images is True:
            for idx in range(0, pts_in_image.shape[1]):
                image = cv2.circle(image, (int(pts_in_image[0, idx]), int(pts_in_image[1, idx])), 5, (255, 0, 0), -1)

            cv2.imshow("Lidar to Camera reprojection - collection " + str(collection_key), image)
            cv2.waitKey()

    # Save evaluation data
    if use_annotation is True:
        createJSONFile(eval_file, output_dict)
