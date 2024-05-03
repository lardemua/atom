#!/usr/bin/env python3

"""
Implementation of an ATOM-compatible alternative calibration method described by Li et. al originally written in MATLAB. Original repo: https://github.com/ihtishamaliktk/RWHE-Calib

This method solves the Robot-World/Hand-Eye calibration problem, with the formulation: AX = ZB, where:

A is the transformation from the camera/sensor to the base;
B is the transformation from the camera to the pattern/target (in the paper, this is called "world". However, to be coherent with ATOM, we call it "pattern");

X is the transformation from the base of the robot to the pattern;
Z is the transformation from the gripper/flange/end-effector to the camera

"""

import argparse

from atom_core.dataset_io import loadResultsJSON

def main():
    
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", type=str, required=True,
                    help="Json file containing input dataset.")
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=str,
                    help="A string to be evaluated into a lambda function that receives a collection name as input and "
                    "returns True or False to indicate if the collection should be loaded (and used in the "
                    "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
                    "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    
    args = vars(ap.parse_args())

    json_file = args['json_file']
    collection_selection_function = args['collection_selection_function']

    # Read dataset file
    dataset, json_file = loadResultsJSON(json_file, collection_selection_function)

    ########################################
    # GET A and B matrices to solve AX=ZB #
    ########################################

        


if __name__ == '__main__':
    main()