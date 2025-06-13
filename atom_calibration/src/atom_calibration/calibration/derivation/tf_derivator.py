#!/usr/bin/env python3

import argparse
from copy import deepcopy
import json
import os
import pprint
from typing import Any, Dict, List, Tuple

from atom_calibration.calibration.derivation.derivation_utils import (
    getTFList,
    plotDerivationResults,
    plotIMUData,
    timeStampToFloat,
)
from atom_core.atom import getTransform
from atom_core.dataset_io import addNoiseToInitialGuess
from atom_core.geometry import (
    matrixToTranslationQuaternion,
    translationQuaternionToTransform,
)
from atom_core.naming import generateKey
from atom_core.utilities import atomError
from matplotlib import pyplot as plt
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
import seaborn as sns
import tf


def deriveRotation(
    tf_data_dict: Dict[str, Any], poly_degree: int, neighbourhood_size: int
) -> Dict:
    """
    Angular velocities are calculated from the temporal derivatives of the rotation matrix

    Inputs:
        - tf_data_dict: a dictionary with the following structure:
            tf_data_dict = {
                "t": [t_array],
                "trans": {"x": [x_trans_array], "y": [y_trans_array], "z": [z_trans_array]},
                "quat": ...
                }
        ;
        - poly_degree: the degree of the polynomial functions to fit the rotation data to;
        - neighbourhood_size: number of datapoints to use for the savgol_filter() function.
    Outputs:
        - ang_vel: A list of 3 arrays of angular velocity at each datapoint;
    """

    # Get time values
    dt = tf_data_dict["t"][1] - tf_data_dict["t"][0]

    # R_arr = []
    r_vec_array = []
    # For each datapoint
    for i in range(len(tf_data_dict["t"])):
        quat = [tf_data_dict["quat"][var][i] for var in ["x", "y", "z", "w"]]
        tvec = [tf_data_dict["trans"][var][i] for var in ["x", "y", "z"]]

        # get tf matrix
        M = translationQuaternionToTransform(tvec, quat)

        r = Rotation.from_matrix(M[:3, :3])
        r_vec = r.as_rotvec()
        r_vec_array.append(r_vec)

    r_vec_array = np.array(r_vec_array)
    dr_vec_array = np.zeros_like(r_vec_array)

    for k in range(3):
        series = r_vec_array[:, k]
        deriv = savgol_filter(
            x=series,
            window_length=neighbourhood_size,
            polyorder=poly_degree,
            deriv=1,
            delta=dt,
        )
        dr_vec_array[:, k] = deriv

    # Compute ang_vels
    ang_vels = {"x": [], "y": [], "z": []}
    for k in range(r_vec_array.shape[0]):
        ang_vels["x"].append(dr_vec_array[k, 0])
        ang_vels["y"].append(dr_vec_array[k, 1])
        ang_vels["z"].append(dr_vec_array[k, 2])

    return ang_vels


def deriveTranslation(
    tf_data_dict: Dict[str, Any],
    poly_degree: int,
    neighbourhood_size: int,
) -> Tuple:
    """
    Inputs:
        - tf_data_dict: a dictionary with the following structure:
            tf_data_dict = {
                "t": [t_array],
                "trans": {"x": [x_trans_array], "y": [y_trans_array], "z": [z_trans_array]},
                "quat": ...
                }
        ;
        - poly_degree: the degree of the polynomial functions to fit the translation data to;
        - neighbourhood_size: number of datapoints to use for the savgol_filter() function.
    Outputs:
        - lin_vel: A list of 3 arrays of linear velocity at each datapoint;
        - lin_accel: A list of 3 arrays of linear acceleration at each datapoint.
    """

    dt = tf_data_dict["t"][1] - tf_data_dict["t"][0]

    lin_vel_x = savgol_filter(
        x=tf_data_dict["trans"]["x"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_vel_y = savgol_filter(
        x=tf_data_dict["trans"]["y"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_vel_z = savgol_filter(
        x=tf_data_dict["trans"]["z"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_accel_x = savgol_filter(
        x=tf_data_dict["trans"]["x"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_accel_y = savgol_filter(
        x=tf_data_dict["trans"]["y"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_accel_z = savgol_filter(
        x=tf_data_dict["trans"]["z"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_vels: dict[str, Any] = {"x": lin_vel_x, "y": lin_vel_y, "z": lin_vel_z}
    lin_accels: dict[str, Any] = {
        "x": lin_accel_x,
        "y": lin_accel_y,
        "z": lin_accel_z,
    }

    return lin_vels, lin_accels


def deriveDataset(
    dataset: dict,
    sensor_name: str,
    neighbourhood_size: int,
    poly_degree: int,
    mode: str,
) -> dict:
    """
    Derive for all timestamps corresponding to /tf messages in a dataset.
    Return a dictionary containing the derivation results for each datapoint (mode="continuous") or for each collection (mode="collections").
    """

    # Define from_frame and to_frame, typically "world" and the IMU frame
    from_frame = dataset["calibration_config"]["world_link"]
    to_frame = dataset["sensors"][sensor_name]["calibration_child"]

    # Get list of timestamps to integrate for
    derivation_results = {}

    tf_pool_lst: List[Dict[Any, Any]] = getTFList(dataset=dataset)
    tf_pool_lst_copy = deepcopy(tf_pool_lst)

    # Organize the data in lists for plotting and deriving
    data_dict = {
        "t": [],
        "trans": {"x": [], "y": [], "z": []},
        "quat": {"x": [], "y": [], "z": [], "w": []},
    }

    # To account for noise being input from the calibrate script, the sensor tfs must be replaced by those from a given collection (since they are static, this should not be an issue)

    for tf_pool_idx in range(len(tf_pool_lst_copy)):
        # for tf_pool in tf_pool_lst_copy:
        tf_pool = tf_pool_lst_copy[tf_pool_idx]
        data_dict["t"].append(timeStampToFloat(stamp=tf_pool.pop("stamp")))

        selected_collection_key = list(dataset["collections"].keys())[0]
        transform_key = generateKey(
            parent=dataset["sensors"][sensor_name]["calibration_parent"],
            child=dataset["sensors"][sensor_name]["calibration_child"],
        )

        new_quat = dataset["collections"][selected_collection_key]["transforms"][
            transform_key
        ]["quat"]
        new_trans = dataset["collections"][selected_collection_key]["transforms"][
            transform_key
        ]["trans"]

        # Now that we have the new translation and rotation values with noise, apply them to all datapoints
        for tf_key, transform in tf_pool.items():
            if transform["child"] == to_frame:
                transform["quat"] = new_quat
                transform["trans"] = list(new_trans)

        # Get source-target tf
        source_target_tf = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_pool
        )

        tvec, quat = matrixToTranslationQuaternion(matrix=source_target_tf)

        data_dict["trans"]["x"].append(tvec[0][0])
        data_dict["trans"]["y"].append(tvec[1][0])
        data_dict["trans"]["z"].append(tvec[2][0])

        data_dict["quat"]["w"].append(quat[0])
        data_dict["quat"]["x"].append(quat[1])
        data_dict["quat"]["y"].append(quat[2])
        data_dict["quat"]["z"].append(quat[3])

    # print(f"source_target_tf_trans_x: {data_dict['trans']['x'][700]}")

    lin_vels, lin_accels = deriveTranslation(
        tf_data_dict=data_dict,
        poly_degree=poly_degree,
        neighbourhood_size=neighbourhood_size,
    )

    ang_vels = deriveRotation(
        tf_data_dict=data_dict,
        poly_degree=poly_degree,
        neighbourhood_size=neighbourhood_size,
    )

    if mode == "continuous":
        derivation_results = {
            "lin_accel": lin_accels,
            "lin_vel": lin_vels,
            "ang_vel": ang_vels,
        }

    elif mode == "collections":

        derivation_results = {}

        for collection_key, collection in dataset["collections"].items():
            # Need to match the collection with the correspondent lin_accel, lin_vel and ang_vel values from the derivation. Use the closest datapoint to make the matching.
            collection_stamp = collection["data"][sensor_name]["header"]["stamp"]
            collection_t = timeStampToFloat(collection_stamp)

            closest_t = min(
                data_dict["t"],
                key=lambda t: abs(t - collection_t),
            )

            closest_t_idx = data_dict["t"].index(closest_t)

            derivation_results[collection_key] = {
                "lin_accel": {
                    "x": lin_accels["x"][closest_t_idx],
                    "y": lin_accels["y"][closest_t_idx],
                    "z": lin_accels["z"][closest_t_idx],
                },
                "lin_vel": {
                    "x": lin_vels["x"][closest_t_idx],
                    "y": lin_vels["y"][closest_t_idx],
                    "z": lin_vels["z"][closest_t_idx],
                },
                "ang_vel": {
                    "x": ang_vels["x"][closest_t_idx],
                    "y": ang_vels["y"][closest_t_idx],
                    "z": ang_vels["z"][closest_t_idx],
                },
            }

    return derivation_results


def calculateErrorsAtCollections(
    dataset: dict, results: dict, sensor_name: str, from_frame: str, to_frame: str
) -> dict:
    """Calculate the errors in the derivation at each collection's timestamp by comparing the derivation results to the sensor data."""

    # Error dict
    e = {}

    for collection_key, result in results.items():

        # Calculate IMU data
        # I need to apply the rotation from the IMU to the world frame to the data from the IMU to compare correctly

        imu_lin_accel = [
            dataset["collections"][collection_key]["data"][sensor_name][
                "linear_acceleration"
            ]["x"],
            dataset["collections"][collection_key]["data"][sensor_name][
                "linear_acceleration"
            ]["y"],
            dataset["collections"][collection_key]["data"][sensor_name][
                "linear_acceleration"
            ]["z"],
        ]

        imu_ang_vel = [
            dataset["collections"][collection_key]["data"][sensor_name][
                "angular_velocity"
            ]["x"],
            dataset["collections"][collection_key]["data"][sensor_name][
                "angular_velocity"
            ]["y"],
            dataset["collections"][collection_key]["data"][sensor_name][
                "angular_velocity"
            ]["z"],
        ]

        world_imu_tf = getTransform(
            from_frame=from_frame,
            to_frame=to_frame,
            transforms=dataset["collections"][collection_key]["transforms"],
        )

        R = world_imu_tf[:3, :3]

        imu_lin_accel = R @ imu_lin_accel

        # Remove gravity
        imu_lin_accel[2] -= 9.81

        e[collection_key] = {
            "lin_accel": {
                "x": abs(imu_lin_accel[0] - result["lin_accel"]["x"]),
                "y": abs(imu_lin_accel[1] - result["lin_accel"]["y"]),
                "z": abs(imu_lin_accel[2] - result["lin_accel"]["z"]),
            },
            "ang_vel": {
                "x": abs(imu_ang_vel[0] - result["ang_vel"]["x"]),
                "y": abs(imu_ang_vel[1] - result["ang_vel"]["y"]),
                "z": abs(imu_ang_vel[2] - result["ang_vel"]["z"]),
            },
        }

    return e


def calculateErrorsAllDataPoints(
    dataset: dict,
    tf_list: List[Dict],
    results: dict,
    sensor_topic: str,
    from_frame: str,
    to_frame: str,
) -> dict:
    """Calculate the errors in the derivation at each tf message timestamp by comparing the derivation results to the closest IMU datapoint. Plot them out."""

    data_dict = {
        "t": [],
        "t_reparam": [],
        "lin_accel_imu": {"x": [], "y": [], "z": []},
        "ang_vel_imu": {"x": [], "y": [], "z": []},
        "e_lin_accel": {"x": [], "y": [], "z": []},
        "e_ang_vel": {"x": [], "y": [], "z": []},
    }

    for i in range(len(tf_list)):
        tf_pool = tf_list[i]

        # Find the closest IMU datapoint
        tf_pool_stamp = tf_pool.pop("stamp")  # Remove stamp so getTransform() works
        tf_pool_t = timeStampToFloat(tf_pool_stamp)

        # Compensate for world-imu tf
        world_imu_tf = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_pool
        )

        # Get closest acceleration data
        closest_sensor_datapoint = min(
            dataset["continuous_data"][sensor_topic],
            key=lambda datapoint: abs(
                timeStampToFloat(datapoint["header"]["stamp"]) - tf_pool_t
            ),
        )

        # Now that we have the closest datapoint, we can compare
        imu_accel = [
            closest_sensor_datapoint["linear_acceleration"]["x"],
            closest_sensor_datapoint["linear_acceleration"]["y"],
            closest_sensor_datapoint["linear_acceleration"]["z"],
        ]
        imu_ang_vel = [
            closest_sensor_datapoint["angular_velocity"]["x"],
            closest_sensor_datapoint["angular_velocity"]["y"],
            closest_sensor_datapoint["angular_velocity"]["z"],
        ]

        R = world_imu_tf[:3, :3]

        imu_accel = R @ imu_accel

        # Remove gravity
        imu_accel[2] -= 9.81

        # For plotting
        data_dict["t"].append(tf_pool_t)
        # Reparametrize t
        data_dict["t_reparam"].append(tf_pool_t - data_dict["t"][0])

        data_dict["lin_accel_imu"]["x"].append(imu_accel[0])
        data_dict["lin_accel_imu"]["y"].append(imu_accel[1])
        data_dict["lin_accel_imu"]["z"].append(imu_accel[2])

        # Calculate errors
        data_dict["e_lin_accel"]["x"].append(
            abs(imu_accel[0] - results["lin_accel"]["x"][i])
        )
        data_dict["e_lin_accel"]["y"].append(
            abs(imu_accel[1] - results["lin_accel"]["y"][i])
        )
        data_dict["e_lin_accel"]["z"].append(
            abs(imu_accel[2] - results["lin_accel"]["z"][i])
        )
        data_dict["e_ang_vel"]["x"].append(
            abs(imu_ang_vel[0] - results["ang_vel"]["x"][i])
        )
        data_dict["e_ang_vel"]["y"].append(
            abs(imu_ang_vel[1] - results["ang_vel"]["y"][i])
        )
        data_dict["e_ang_vel"]["z"].append(
            abs(imu_ang_vel[2] - results["ang_vel"]["z"][i])
        )

    e = {"lin_accel": data_dict["e_lin_accel"], "ang_vel": data_dict["e_ang_vel"]}

    return e


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    # ap.add_argument(
    #     "-m",
    #     "--mode",
    #     type=str,
    #     default="collections",
    #     help="Choose whether to plot out the errors align the entire dataset or only calculate the errors at each collection. Accepted modes are: ['dataset', 'collections']",
    # )
    ap.add_argument(
        "-json",
        "--json_file",
        type=str,
        required=True,
        help="Json file containing input dataset.",
    )
    ap.add_argument(
        "-ns",
        "--neighbourhood_size",
        type=int,
        default=75,
        help="Number of TF samples to use for curve-fitting at each datapoint.",
    )
    ap.add_argument(
        "-pd",
        "--poly_degree",
        type=int,
        default=3,
        help="Degree of polynomial to use for curve-fitting.",
    )
    ap.add_argument(
        "-sdp",
        "--save_derivation_plot",
        help="Store the results in a plot when deriving the entire dataset",
        action="store_true",
        default=False,
    )
    ap.add_argument(
        "-nig",
        "--noisy_initial_guess",
        nargs=2,
        metavar=("translation", "rotation"),
        help="Magnitude of noise to add to the initial guess atomic transformations set before starting optimization [meters, radians].",
        type=float,
        default=[0.0, 0.0],
    )
    ap.add_argument(
        "-dm",
        "--derivation_mode",
        type=str,
        required=False,
        default="collections",
        help="Decides whether derivation errors are calculated at each collection or in a continuous manner, throughout the dataset.",
    )
    ap.add_argument("-ss", "--sample_seed", help="Sampling seed", type=int)

    args = vars(ap.parse_args())
    neighbourhood_size = args["neighbourhood_size"]

    # Verify that "mode" is valid
    if (
        args["derivation_mode"] != "continuous"
        and args["derivation_mode"] != "collections"
    ):
        atomError(message="Invalid value for derivation mode!")

    # Find dataset name for results saving purposes
    dataset_name = args["json_file"].split("/")[-2]

    with open(args["json_file"]) as f:
        input_dataset = json.load(f)

    # Add a grid in the background of the graphs
    sns.set_theme(style="whitegrid")

    dataset_ground_truth = deepcopy(x=input_dataset)

    selected_collection_key = list(input_dataset["collections"].keys())[0]

    addNoiseToInitialGuess(input_dataset, args, selected_collection_key)

    tf_lst = getTFList(input_dataset)

    derivation_results = deriveDataset(
        dataset=input_dataset,
        sensor_name="imu_chassis",
        neighbourhood_size=neighbourhood_size,
        poly_degree=args["poly_degree"],
        mode=args["derivation_mode"],
    )


    if args["derivation_mode"] == "collections":
        e = calculateErrorsAtCollections(
            dataset=input_dataset,
            results=derivation_results,
            sensor_name="imu_chassis",
            from_frame="world",
            to_frame="accelerometer",
        )

    elif args["derivation_mode"] == "continuous":
        plotDerivationResults(
            dataset=input_dataset,
            tf_list=tf_lst,
            derivation_results=derivation_results,
            from_frame="world",
            to_frame="accelerometer",
            noise=args["noisy_initial_guess"],
            dataset_name=dataset_name,
        )
        e = calculateErrorsAllDataPoints(
            dataset=input_dataset,
            tf_list=tf_lst,
            results=derivation_results,
            sensor_topic="/imu",
            from_frame="world",
            to_frame="accelerometer",
        )
    
