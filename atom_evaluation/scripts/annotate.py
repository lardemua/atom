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
import ros_numpy

import atom_core.atom
from atom_core.dataset_io import getPointCloudMessageFromDictionary, read_pcd

from rospy_message_converter import message_converter
import cv2
import argparse
import OptimizationUtils.utilities as opt_utilities
from scipy.spatial import distance
from copy import deepcopy
from colorama import Style, Fore
from collections import OrderedDict

from atom_core.naming import generateKey


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
    print (json.dumps(D, indent=2, sort_keys=True), file=f)
    f.close()
    print("Completed.")



mouseX, mouseY = 0, 0
def click(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y
    else:
        mouseX, mouseY = 0, 0


def annotateLimits(image):
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', click)

    extremas = {}
    [extremas.setdefault(x, []) for x in range(4)]
    colors = [(125, 125, 125), (0, 255, 0), (0, 0, 255), (125, 0, 125)]
    annotating = True
    i = 0
    p_mouseX, p_mouseY = 0, 0
    while i < 4:
        cv2.imshow('image', image)
        k = cv2.waitKey(20) & 0xFF
        if k == ord('d'):
            cv2.destroyWindow('image')
            return [], False
        elif k == ord('c'):
            i += 1
        else:
            if (mouseX != 0 and mouseY != 0) and (p_mouseX != mouseX and p_mouseY != mouseY):
                image = cv2.circle(image, (mouseX, mouseY), 5, colors[i], -1)
                extremas[i].append([mouseX, mouseY])

        p_mouseX = mouseX
        p_mouseY = mouseY

    cv2.destroyWindow('image')
    return extremas, True


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-test_json", "--test_json_file", help="Json file containing input testing dataset.", type=str,
                    required=True)
    ap.add_argument("-cs", "--camera_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=True)
    ap.add_argument("-ef", "--eval_file", help="Path to file to read and/or write the evalutation data.", type=str,
                    required=True)


    # - Save args
    args = vars(ap.parse_args())
    # source_sensor = args['source_sensor']
    camera_sensor = args['camera_sensor']
    show_images = args['show_images']
    eval_file = args['eval_file']
    # use_annotation = args['use_annotation']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    # train_json_file = args['train_json_file']
    # f = open(train_json_file, 'r')
    # train_dataset = json.load(f)
    test_json_file = args['test_json_file']
    f = open(test_json_file, 'r')
    test_dataset = json.load(f)


    print(Fore.BLUE + "  Annotation tool intructions:")
    print(Fore.GREEN + "   - To add a point to a class: click")
    print(Fore.GREEN + "   - To change class: 'c'")
    print(Fore.GREEN + "   - To stop the annotation anytime: 'd'")
    print(Fore.GREEN + "   - It ends when you end annotating the fourth class (four times 'p')")
    print(Fore.WHITE)

    # Declare output dict to save the evaluation data if desired
    output_dict = {}
    output_dict['ground_truth_pts'] = {}

    delta_total = []

    from_frame = test_dataset['calibration_config']['sensors'][camera_sensor]['link']
    od = OrderedDict(sorted(test_dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Get evaluation data for current collection
        # ---------------------------------------
        filename = os.path.dirname(test_json_file) + '/' + collection['data'][camera_sensor]['data_file']
        print (filename)
        image = cv2.imread(filename)

        success = False
        while not success:
            limits_on_image, success = annotateLimits(image)
            if not success:
                limits_on_image = []
                image = cv2.imread(filename)

        # Clear image annotations
        image = cv2.imread(filename)

        output_dict['ground_truth_pts'][collection_key] = {}
        for i, pts in limits_on_image.items():
            pts = np.array(pts)
            if pts.size == 0:
                continue

            x = pts[:, 0]
            y = pts[:, 1]
            coefficients = np.polyfit(x, y, 3)
            poly = np.poly1d(coefficients)
            new_x = np.linspace(np.min(x), np.max(x), 5000)
            new_y = poly(new_x)

            if show_images:
                for idx in range(0, len(new_x)):
                    image = cv2.circle(image, (int(new_x[idx]), int(new_y[idx])), 3, (0, 0, 255), -1)

            output_dict['ground_truth_pts'][collection_key][i] = []
            for idx in range(0, len(new_x)):
                output_dict['ground_truth_pts'][collection_key][i].append([new_x[idx], new_y[idx]])


    createJSONFile(eval_file, output_dict)
    print('Anotated json file created.')
