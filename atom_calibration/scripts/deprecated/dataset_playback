#!/usr/bin/env python3
import argparse
from json_reader import *
import os, math, signal, sys
import cv2
import numpy as np

# 3rd-party
import rospy, tf
from colorama import Fore, Style, Back
from urdf_parser_py.urdf import URDF

import atom_calibration.calibration.patterns_config as patterns
from atom_core.naming import generateKey, generateName
from atom_calibration.calibration.visualization import setupVisualization, visualizationFunction
from atom_core.dataset_io import loadResultsJSON, saveResultsJSON, \
    getPointCloudMessageFromDictionary, filterCollectionsFromDataset, filterSensorsFromDataset, \
    addNoiseToInitialGuess

# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    # ap = OptimizationUtils.addArguments(ap)  # OptimizationUtils arguments
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-ajf", "--all_joints_fixed",
                    help="Assume all joints are fixed and because of that draw a single robot mesh. Overrides "
                         "automatic detection of static robot.",
                    action='store_true', default=False)
    ap.add_argument("-uic", "--use_incomplete_collections",
                    help="Remove any collection which does not have a detection for all sensors.",
                    action='store_true', default=False)
    ap.add_argument("-rpd", "--remove_partial_detections",
                    help="Remove detected labels which are only partial. Used or the Charuco.",
                    action='store_true', default=False)
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=lambda s: eval(s, globals()),
                    help='A string to be evaluated into a lambda function that receives a collection name as input and '
                         'returns True or False to indicate if the collection should be loaded (and used in the '
                         'optimization). The Syntax is lambda name: f(x), where f(x) is the function in python '
                         'language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.')
    ap.add_argument("-ssf", "--sensor_selection_function",  default=None, type=lambda s: eval(s, globals()), help='A')
    ap.add_argument("-ipg", "--initial_pose_ghost",
                    help="Draw a ghost mesh with the systems initial pose. Good for debugging.",
                    action='store_true', default=False)

    # Roslaunch adds two arguments (__name and __log) that break our parser. Lets remove those.
    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(ap.parse_args(args=arglist))

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    # Loads a json file containing the detections. Returned json_file has path resolved by urireader.

    dataset, json_file = loadResultsJSON(args['json_file'], args['collection_selection_function'])

    # ---------------------------------------
    # --- Filter some collections
    # ---------------------------------------
    dataset = filterCollectionsFromDataset(dataset, args)

    # filter collections
    # Create the chessboard dataset must be called before deleting the sensors to cope with the possibility of
    # setting up an optimization without cameras. For now we MUST have a camera to estimate the initial parameters
    # related to the pattern pose (we use solve PNP for a camera).
    dataset['patterns'] = patterns.createPatternLabels(args, dataset)  # TODO: Solve this strange dependency.

    # ---------------------------------------
    # --- Filter some sensors from the dataset
    # ---------------------------------------
    dataset = filterSensorsFromDataset(dataset, args) # filter sensors

    print('Loaded dataset containing ' + str(len(dataset['sensors'].keys())) + ' sensors and ' + str(
        len(dataset['collections'].keys())) + ' collections.')

    # exit(0)
    # models = dataset
    # args = models['args']
    # collections = models['dataset']['collections']

    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # For the getters we only need to get one collection. Lets take the first key on the dictionary and always get that
    # transformation.
    selected_collection_key = list(dataset['collections'].keys())[0]
    print('Selected collection key is ' + str(selected_collection_key))

    # ---------------------------------------
    # --- DEFINE THE VISUALIZATION FUNCTION
    # ---------------------------------------
    graphics = setupVisualization(dataset, args, selected_collection_key)
    models={'dataset': dataset, 'graphics': graphics, 'args': args}

    visualizationFunction(models)


if __name__ == "__main__":
    main()
