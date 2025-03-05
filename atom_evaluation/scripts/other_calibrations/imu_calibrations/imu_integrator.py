#!/usr/bin/env python3

import argparse
from copy import deepcopy
import sys

from matplotlib import pyplot as plt
import numpy as np
import seaborn
import pandas as pd

from atom_calibration.collect import patterns
from atom_core.atom import getTransform
from atom_core.dataset_io import addNoiseToInitialGuess, filterCollectionsFromDataset, loadResultsJSON
from atom_core.utilities import atomError, createLambdaExpressionsForArgs
from atom_core.geometry import matrixToTranslationQuaternion, matrixToTranslationRotation

def quat_mult(q, r):
    """Hamilton product of two quaternions"""
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = r
    
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_derivative(q, omega):
    """Compute quaternion derivative using angular velocity"""
    omega_quat = np.array([0, *omega])  # Convert angular velocity to pure quaternion
    dqdt = 0.5 * quat_mult(q, omega_quat)
    return dqdt

def skew_sym_matrix(vector):

    skew_symmetric_matrix = np.zeros((3,3))

    skew_symmetric_matrix[0,1] = -vector[2]
    skew_symmetric_matrix[0,2] = vector[1]
    skew_symmetric_matrix[1,0] = vector[2]
    skew_symmetric_matrix[1,2] = -vector[0]
    skew_symmetric_matrix[2,0] = -vector[1]
    skew_symmetric_matrix[2,1] = vector[0]

    return skew_symmetric_matrix

def omega(vector):

    M = np.zeros((4,4))
    M[1:,1:] = -skew_sym_matrix(vector)
    M[1:, 0] = vector
    M[0, 1:] = -vector
    
    return M

def normalize_quaternion(quaternion):
    
    if quaternion[0] < 0:
        quaternion = -1 * quaternion
    
    normalized_quaternion = quaternion/np.linalg.norm(quaternion)

    return normalized_quaternion

def rk4_imu_integration(imu_data_0, imu_data_1, q):
    # This function receives two imu "data points" and integrates the angular velocity and linear acceleration to calculate the angular and linear displacements between these two points.

    # Get delta_t
    t_0 = imu_data_0["header"]["stamp"]["secs"] + (10**(-9)) * imu_data_0["header"]["stamp"]["nsecs"]
    t_1 = imu_data_1["header"]["stamp"]["secs"] + (10**(-9)) * imu_data_1["header"]["stamp"]["nsecs"]
    delta_t = t_1 - t_0

    omega = np.array([*imu_data_0["angular_velocity"].values()])
    
    k1 = quat_derivative(q, omega)*delta_t
    k2 = quat_derivative(q + k1/2, omega)*delta_t
    k3 = quat_derivative(q + k2/2, omega)*delta_t
    k4 = quat_derivative(q + k3, omega)*delta_t

    q_new = q + (k1 + 2*k2 + 2*k3 + k4) / 6
    q_new = normalize_quaternion(q_new)
    return q_new

def euler_integration(imu_data_0, imu_data_1, q):
    # A simplistic integration method

    t_0 = imu_data_0["header"]["stamp"]["secs"] + (10**(-9)) * imu_data_0["header"]["stamp"]["nsecs"]
    t_1 = imu_data_1["header"]["stamp"]["secs"] + (10**(-9)) * imu_data_1["header"]["stamp"]["nsecs"]
    delta_t = t_1 - t_0

    initial_delta_orientation = q
    
    omega = np.array([*imu_data_0["angular_velocity"].values()])

    # Get quat derivative
    dqdt = 0.5 * np.array([
        -initial_delta_orientation[1] * omega[0] - initial_delta_orientation[2] * omega[1] - initial_delta_orientation[3] * omega[2],
        initial_delta_orientation[0] * omega[0] + initial_delta_orientation[2] * omega[2] - initial_delta_orientation[3] * omega[1],
        initial_delta_orientation[0] * omega[1] - initial_delta_orientation[1] * omega[2] + initial_delta_orientation[3] * omega[0],
        initial_delta_orientation[0] * omega[2] + initial_delta_orientation[1] * omega[1] - initial_delta_orientation[2] * omega[0]
    ])

    delta_orientation = initial_delta_orientation + dqdt*delta_t
    delta_orientation = normalize_quaternion(delta_orientation)

    return delta_orientation


def main():
    ########################################
    # ARGUMENT PARSER #
    ########################################

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", type=str, required=True, help="Json file containing input dataset.")
    ap.add_argument("-imu", "--imu_name", help="The name of the IMU sensor.", type=str, required=True)
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=str, help="A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    ap.add_argument("-uic", "--use_incomplete_collections", action="store_true", default=False, help="Remove any collection which does not have a detection for all sensors.")
    ap.add_argument("-mn", "--method_name", help="The method to use for integration. Available options: [rk4, euler]", required=True)
    
    # Roslaunch adds two arguments (__name and __log) that break our parser. Lets remove those.
    arglist = [x for x in sys.argv[1:] if not x.startswith("__")]
    # these args have the selection functions as strings
    args_original = vars(ap.parse_args(args=arglist))
    args = createLambdaExpressionsForArgs(args_original)  # selection functions are now lambdas

    json_file = args['json_file']
    imu_name = args["imu_name"]
    collection_selection_function = args["collection_selection_function"]
    method_name = args["method_name"]

    # Read dataset file
    dataset, json_file = loadResultsJSON(json_file, collection_selection_function)

    dataset_ground_truth = deepcopy(dataset)  # make a copy before adding noise

    # ---------------------------------------
    # --- Validations 
    # ---------------------------------------

    if method_name not in ["rk4", "euler"]:
        atomError("Invalid method name argument! Please use either 'rk4' or 'euler'!")   


    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # We only need to get one collection because optimized transformations are static, which means they are the same for all collections. Let's select the first key in the dictionary and always get that transformation.
    selected_collection_key = list(dataset["collections"].keys())[0]
    print("Selected collection key is " + str(selected_collection_key))

    # ---------------------------------------
    # --- Implementation
    # ---------------------------------------
    
    

    # For each collection, get a list of all IMU data from continuous_sensor_data from the previous collection to the next 
    
    tmp_tf = getTransform(
        from_frame="world",
        to_frame="imu_link",
        transforms=dataset["collections"]["000"]["transforms"]
    )
    # 
    # print(tmp_tf)
    # exit(0)
    # tmp_q = np.array([0.0000143, 0.0003491, 0.9999999, 0.0])

    tmp_checkpoint = 0 # Here to avoid iterating over the same datapoints
    for collection_key, collection in dataset["collections"].items():

        # If its the first collection, get an initial value for the orientation
        if collection_key == list(dataset["collections"].keys())[0]:
            tmp_tf = getTransform(
                from_frame="world",
                to_frame="imu_link",
                transforms=collection["transforms"]
                )

            tmp_t, tmp_q = matrixToTranslationQuaternion(tmp_tf)            
            # tmq_q = quat_mult(np.array([*collection["data"][imu_name]["orientation"].values()]), tmp_q) 

            # print(tmp_q)
            continue

        collection_stamp = (collection["data"][imu_name]["header"]["stamp"]["secs"], collection["data"][imu_name]["header"]["stamp"]["nsecs"])


        for i in range(tmp_checkpoint, len(dataset["continuous_sensor_data"][imu_name])-1):

            # if i == 0:
            #     initial_orientation = np.array([*dataset["continuous_sensor_data"][imu_name][i]["orientation"].values()])
            #     tmp_q = initial_orientation
            # if i%2 != 0:
                # continue

            data_0 = dataset["continuous_sensor_data"][imu_name][i]
            data_1 = dataset["continuous_sensor_data"][imu_name][i+1]

            if method_name == "rk4":
                tmp_q = rk4_imu_integration(
                    imu_data_0=data_0,
                    imu_data_1=data_1,
                    q = tmp_q
                )
            
            elif method_name == "euler":
                tmp_q = euler_integration(
                    imu_data_0=data_0,
                    imu_data_1=data_1,
                    q = tmp_q
                )

            if (dataset["continuous_sensor_data"][imu_name][i]["header"]["stamp"]["secs"], dataset["continuous_sensor_data"][imu_name][i]["header"]["stamp"]["nsecs"]) ==  collection_stamp:
                tmp_checkpoint = i

                gt_tf=getTransform(
                    from_frame="world",
                    to_frame="imu_link",
                    transforms=collection["transforms"]
                )

                gt_t, gt_quat = matrixToTranslationQuaternion(gt_tf)
                
                print(f"gt_quat_imu_link = {gt_quat}")

                print(f"Integrator quat = {tmp_q}")
# 
                print(np.linalg.norm(gt_quat - tmp_q))

   

if __name__ == "__main__":
    main()