"""
Utilities for the derivation of TF data
"""

import argparse
import json
import os
import pathlib
from copy import deepcopy
from math import floor
from pprint import pprint
from typing import Any, Dict, List

import numpy as np
import seaborn as sns
from atom_core.atom import getTransform
from atom_core.geometry import (
    matrixToTranslationQuaternion,
    matrixToTranslationRotation,
)
from atom_core.utilities import atomError
from matplotlib import pyplot as plt
from prettytable import PrettyTable
from scipy.spatial.transform import Rotation


def timeFloatToStamp(t_float: float) -> Dict[str, int]:

    secs = floor(t_float)
    nsecs = round((t_float - secs) * 10**9)

    stamp = {"secs": secs, "nsecs": nsecs}

    return stamp


def timeStampToFloat(stamp: Dict[str, int]) -> float:

    t_float = stamp["secs"] + (10 ** (-9)) * stamp["nsecs"]

    return t_float


def quatMult(q: List, p: List) -> List:

    res = [
        p[0] * q[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3],
        q[1] * p[0] + q[0] * p[1] + q[2] * p[3] - q[3] * p[2],
        q[2] * p[0] + q[0] * p[2] + q[3] * p[1] - q[1] * p[3],
        q[3] * p[0] + q[0] * p[3] + q[1] * p[2] - q[2] * p[1],
    ]

    return res


def convertRotationsInTFToEuler(tf_list: List[Dict]) -> List[Dict]:
    """Convert the rotations in a list of TFs to euler angles (XYZ convention)"""

    for tf in tf_list:
        r = Rotation.from_quat(tf["quat"])
        r_euler = r.as_euler(seq="XYZ")

        tf["euler"] = r_euler.tolist()

    return tf_list


def deriveRotation(dataset: dict, visualization: bool) -> dict:
    """Given a list of transformations, return the functions that describe the angular velocities.
    Angular velocities are calculated using the following:

        dw = 2 * quaternionMultiplication(dq, q), with dw = [0, wx, wy, wz]

    Inputs:
        - dataset: dataset
        - visualization: enable graph visualization.
    Outputs:
        - ang_vel_dict: a dictionary with the angular velocities over time.
    """

    # Get time values
    t_arr = []
    quat_dict = {"x": [], "y": [], "z": [], "w": []}

    for datapoint in dataset["continuous_data"]["/tf"]:
        # Get time
        t_arr.append(timeStampToFloat(datapoint["transforms"][0]["header"]["stamp"]))

        # Get all tfs for the tf_pool
        tf_dict = {}

        for tf in datapoint["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{parent_frame}-{child_frame}"

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": [*tf["transform"]["rotation"].values()],
                "trans": [*tf["transform"]["translation"].values()],
            }

        # Include transforms from /tf_static. Only consider the last message.
        for tf in dataset["continuous_data"]["/tf_static"][-1]["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{tf['header']['frame_id']}-{tf['child_frame_id']}"

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": [*tf["transform"]["rotation"].values()],
                "trans": [*tf["transform"]["translation"].values()],
            }

        # Now get the tf from the source to the target frames
        source_to_target_tf_trans, source_to_target_tf_quat = (
            matrixToTranslationQuaternion(
                getTransform(
                    from_frame="world", to_frame="imu_link", transforms=tf_dict
                )
            )
        )

        quat_dict["w"].append(source_to_target_tf_quat[0])
        quat_dict["x"].append(source_to_target_tf_quat[1])
        quat_dict["y"].append(source_to_target_tf_quat[2])
        quat_dict["z"].append(source_to_target_tf_quat[3])

    ang_vel_dict = {"x": [], "y": [], "z": []}

    for i in range(len(t_arr)):
        if i == 0 or i == len(t_arr) - 1:
            continue

        data = {
            "t": [t_arr[i - 1], t_arr[i], t_arr[i + 1]],
            "w": [quat_dict["w"][i - 1], quat_dict["w"][i], quat_dict["w"][i + 1]],
            "x": [quat_dict["x"][i - 1], quat_dict["x"][i], quat_dict["x"][i + 1]],
            "y": [quat_dict["y"][i - 1], quat_dict["y"][i], quat_dict["y"][i + 1]],
            "z": [quat_dict["z"][i - 1], quat_dict["z"][i], quat_dict["z"][i + 1]],
        }

        ddata_dt = centralNumericalFirstDerivative(data)

        # Now that we have dq, we need to convert it to omegas
        q_conjugate = [
            quat_dict["w"][i],
            -1 * quat_dict["x"][i],
            -1 * quat_dict["y"][i],
            -1 * quat_dict["z"][i],
        ]

        omega = quatMult(2 * ddata_dt, q_conjugate)

        ang_vel_dict["x"].append(omega[1])
        ang_vel_dict["y"].append(omega[2])
        ang_vel_dict["z"].append(omega[3])

    if visualization:
        fig, axes = plt.subplots(1, 3)

        plot_titles = [
            r"$\omega_{x}(t)$",
            r"$\omega_{y}(t)$",
            r"$\omega{z}(t)$",
        ]

        for ax, title in zip(axes, plot_titles):
            ax.set_title(title)

        t_arr_plot = t_arr[1:-1]

        sns.scatterplot(x=t_arr_plot, y=ang_vel_dict["x"], ax=axes[0])
        sns.scatterplot(x=t_arr_plot, y=ang_vel_dict["y"], ax=axes[1])
        sns.scatterplot(x=t_arr_plot, y=ang_vel_dict["z"], ax=axes[2])

        plt.show()

    return ang_vel_dict


def centralNumericalFirstDerivative(data: Dict[str, float]) -> List[float]:

    ddata_dt = []

    for var in data.keys():
        if var == "t":
            continue

        dvar_dt = (data[var][2] - data[var][0]) / (data["t"][2] - data["t"][0])
        ddata_dt.append(dvar_dt)

    return ddata_dt


def centralNumericalSecondDerivative(data: Dict[str, float]) -> List[float]:

    dddata_dtt = []

    for var in data.keys():
        if var == "t":
            continue

        ddvar_dtt = (data[var][2] - 2 * data[var][1] + data[var][0]) / (
            (data["t"][2] - data["t"][1]) ** 2
        )
        dddata_dtt.append(ddvar_dtt)

    return dddata_dtt


def deriveTranslation(dataset: dict, visualization: bool) -> dict:
    """Given a list of transformations, return the functions that describe the linear accelerations.

    Inputs:
        - dataset
        - visualization: enable graph visualization.
    Outputs:
        - lin_accel_dict: a dictionary with the linear acceleration values over time.
    """

    # Get time values
    t_arr = []
    trans_dict = {"x": [], "y": [], "z": []}

    for datapoint in dataset["continuous_data"]["/tf"]:
        # Get time
        t_arr.append(timeStampToFloat(datapoint["transforms"][0]["header"]["stamp"]))

        # Get all tfs for the tf_pool
        tf_dict = {}

        for tf in datapoint["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{parent_frame}-{child_frame}"

            trans = [
                tf["transform"]["translation"]["x"],
                tf["transform"]["translation"]["y"],
                tf["transform"]["translation"]["z"],
            ]
            quat = [
                tf["transform"]["rotation"]["x"],
                tf["transform"]["rotation"]["y"],
                tf["transform"]["rotation"]["z"],
                tf["transform"]["rotation"]["w"],
            ]

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": quat,
                "trans": trans,
            }

        # Include transforms from /tf_static. Only consider the last message.
        for tf in dataset["continuous_data"]["/tf_static"][-1]["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{tf['header']['frame_id']}-{tf['child_frame_id']}"

            trans = [
                tf["transform"]["translation"]["x"],
                tf["transform"]["translation"]["y"],
                tf["transform"]["translation"]["z"],
            ]
            quat = [
                tf["transform"]["rotation"]["x"],
                tf["transform"]["rotation"]["y"],
                tf["transform"]["rotation"]["z"],
                tf["transform"]["rotation"]["w"],
            ]

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": quat,
                "trans": trans,
            }

        # Now get the tf from the source to the target frames
        source_to_target_tf_trans, source_to_target_tf_quat = (
            matrixToTranslationQuaternion(
                getTransform(
                    from_frame="world", to_frame="imu_link", transforms=tf_dict
                )
            )
        )

        trans_dict["x"].append(source_to_target_tf_trans[0][0])
        trans_dict["y"].append(source_to_target_tf_trans[1][0])
        trans_dict["z"].append(source_to_target_tf_trans[2][0])

    lin_accel_dict = {"x": [], "y": [], "z": []}

    for i in range(len(t_arr)):
        if (i == 0 or i == 1) or (i == len(t_arr) - 1 or i == len(t_arr) - 2):
            continue

        data = {
            "t": [t_arr[i - 1], t_arr[i], t_arr[i + 1]],
            "x": [trans_dict["x"][i - 1], trans_dict["x"][i], trans_dict["x"][i + 1]],
            "y": [trans_dict["y"][i - 1], trans_dict["y"][i], trans_dict["y"][i + 1]],
            "z": [trans_dict["z"][i - 1], trans_dict["z"][i], trans_dict["z"][i + 1]],
        }

        dddata_dtt = centralNumericalSecondDerivative(data)

        lin_accel_dict["x"].append(dddata_dtt[0])
        lin_accel_dict["y"].append(dddata_dtt[1])
        lin_accel_dict["z"].append(dddata_dtt[2])

    if visualization:
        
        fig, ax1 = plt.subplots()

        plt.title("Displacement and Linear Acceleration")

        t_arr_plot = t_arr[2:-2]
        t_arr_reparam = [t - t_arr[0] for t in t_arr]
        t_arr_plot_reparam = [t - t_arr[0] for t in t_arr_plot]

        ax2 = ax1.twinx()

        sns.scatterplot(x=t_arr_reparam, y=trans_dict["x"], marker="o", color="red", ax=ax1)
        sns.scatterplot(x=t_arr_reparam, y=trans_dict["y"], marker="o", color="green", ax=ax1)
        sns.scatterplot(x=t_arr_reparam, y=trans_dict["z"], marker="o", color="blue", ax=ax1)
        sns.scatterplot(x=t_arr_plot_reparam, y=lin_accel_dict["x"], marker="x", color="red", ax=ax2)
        sns.scatterplot(x=t_arr_plot_reparam, y=lin_accel_dict["y"], marker="x", color="green", ax=ax2)
        sns.scatterplot(x=t_arr_plot_reparam, y=lin_accel_dict["z"], marker="x", color="blue", ax=ax2)

        plt.show()
        exit(0)

    return lin_accel_dict


def deriveFromTF(
    dataset: dict,
    from_frame: str,
    to_frame: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
    transition_point_list: List,
) -> List[float]:
    """Given a dataset, return the results of derivation."""

    # Don't do anything if t is a transition point
    # if t in transition_point_list:
    #     return None, None, None

    lin_accel_dict = deriveTranslation(dataset=dataset, visualization=visualization)

    ang_vel_dict = deriveRotation(dataset=dataset, visualization=visualization)


def calculateErrorsAtCollections(
    dataset: dict, results: dict, sensor_name: str, from_frame: str, to_frame: str
) -> dict:
    """Calculate the errors in the derivation at each collection's timestamp by comparing the derivation results to the sensor data."""

    # Error dict
    e = {}

    for collection_key, results in results.items():

        # Calculate IMU data
        # I need to apply the rotation from the IMU to the world frame to the data from the IMU to compare correctly
        imu_accel = [
            *dataset["collections"][collection_key]["data"][sensor_name][
                "linear_acceleration"
            ].values()
        ]
        imu_ang_vel = [
            *dataset["collections"][collection_key]["data"][sensor_name][
                "angular_velocity"
            ].values()
        ]

        world_T_imu = getTransform(
            from_frame=from_frame,
            to_frame=to_frame,
            transforms=dataset["collections"][collection_key]["transforms"],
        )

        R = world_T_imu[:3, :3]

        imu_accel = R @ imu_accel

        # Remove gravity
        imu_accel[2] -= 9.81

        e[collection_key] = {
            "e_lin_accel": np.linalg.norm(np.array(imu_accel) - results["lin_accel"]),
            "e_ang_vel": np.linalg.norm(np.array(imu_ang_vel) - results["ang_vel"]),
        }

    return e


def calculateErrorsAllDataPoints(
    dataset: dict,
    lin_accel_dict: dict,
    ang_vel_dict: dict,
    sensor_topic: str,
    from_frame: str,
    to_frame: str,
    save_derivation_plot: bool,
) -> dict:
    """Calculate the errors in the derivation at each tf message timestamp by comparing the derivation results to the closest IMU datapoint. Plot them out."""

    # Error dict with errors vectors for each axis
    e = {
        "e_lin_accel": {"x": [], "y": [], "z": []},
        "e_ang_vel": {"x": [], "y": [], "z": []},
    }
    # Time vector
    t_vec = []

    for i in range(len(dataset["continuous_data"]["/tf"])):

        if i in [
            0,
            1,
            len(dataset["continuous_data"]["/tf"]) - 1,
            len(dataset["continuous_data"]["/tf"]) - 2,
        ]:
            continue

        results_lists_idx = i - 2

        print(results_lists_idx)

        tf_msg = dataset["continuous_data"]["/tf"][i]

        # Find the closest IMU datapoint
        tf_pool_t = timeStampToFloat(tf_msg["transforms"][0]["header"]["stamp"])

        t_dist_min = None
        for sensor_datapoint in dataset["continuous_data"][sensor_topic]:
            sensor_datapoint_t = timeStampToFloat(sensor_datapoint["header"]["stamp"])

            t_dist = abs(tf_pool_t - sensor_datapoint_t)

            if t_dist_min is None or t_dist < t_dist_min:
                t_dist_min = t_dist
                closest_sensor_datapoint = sensor_datapoint

        # Now that we have the closest datapoint, we can compare
        imu_accel = [*closest_sensor_datapoint["linear_acceleration"].values()]
        imu_ang_vel = [*closest_sensor_datapoint["angular_velocity"].values()]

        # Compensate for world-imu tf
        t_vec.append(timeStampToFloat(tf_msg["transforms"][0]["header"]["stamp"]))

        # Get all tfs for the tf_pool
        tf_dict = {}

        for tf in tf_msg["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{parent_frame}-{child_frame}"

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": [*tf["transform"]["rotation"].values()],
                "trans": [*tf["transform"]["translation"].values()],
            }

        # Include transforms from /tf_static. Only consider the last message.
        for tf in dataset["continuous_data"]["/tf_static"][-1]["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{tf['header']['frame_id']}-{tf['child_frame_id']}"

            tf_dict[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": [*tf["transform"]["rotation"].values()],
                "trans": [*tf["transform"]["translation"].values()],
            }

        # Now get the tf from the source to the target frames
        world_imu_tf = getTransform(
            from_frame="world", to_frame="imu_link", transforms=tf_dict
        )

        R = world_imu_tf[:3, :3]

        imu_accel = R @ imu_accel

        # Remove gravity
        imu_accel[2] -= 9.81

        # Calculate errors
        e["e_lin_accel"]["x"].append(
            imu_accel[0] - lin_accel_dict["x"][results_lists_idx]
        )
        e["e_lin_accel"]["y"].append(
            imu_accel[1] - lin_accel_dict["y"][results_lists_idx]
        )
        e["e_lin_accel"]["z"].append(
            imu_accel[2] - lin_accel_dict["z"][results_lists_idx]
        )
        e["e_ang_vel"]["x"].append(
            imu_ang_vel[0] - ang_vel_dict["x"][results_lists_idx]
        )
        e["e_ang_vel"]["y"].append(
            imu_ang_vel[1] - ang_vel_dict["y"][results_lists_idx]
        )
        e["e_ang_vel"]["z"].append(
            imu_ang_vel[2] - ang_vel_dict["z"][results_lists_idx]
        )

    # Reparametrize time
    t_vec_reparam = []
    for i in range(len(t_vec)):
        t_vec_reparam.append(t_vec[i] - t_vec[0])

    print(len(t_vec_reparam))
    
    fig, axes = plt.subplots(2, 1)
    sns.scatterplot(x=t_vec_reparam, y=e["e_lin_accel"]["x"], marker="o", color="red", s=30, label=r"$E_{a_x}$", ax=axes[0])
    sns.scatterplot(x=t_vec_reparam, y=e["e_lin_accel"]["y"], marker="o", color="green", s=30, label=r"$E_{a_y}$", ax=axes[0])
    sns.scatterplot(x=t_vec_reparam, y=e["e_lin_accel"]["z"], marker="o", color="blue", s=30, label=r"$E_{a_z}$", ax=axes[0])
    
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["x"], s=30, label="x", ax=axes[1])
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["y"], s=30, label="y", ax=axes[1])
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["z"], s=30, label="z", ax=axes[1])

    plot_titles = [
        r"$E_{a}(t)$",
        r"$E_{\omega}(t)$",
    ]
    
    for ax, title in zip(axes, plot_titles):
        ax.set_title(title)
    
    axes[0].set(
        xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[ms^{-2}]$"
    )
    axes[1].set(
        xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[rad/s]$"
    )
    
    fig.tight_layout()

    if save_derivation_plot:
        plt.savefig(fname="results.png", dpi=300)
    plt.show()

    return e


def plotIMUData(dataset: Dict) -> None:
    """Simple function for debugging."""

    t_arr = []
    imu_data = {"x": [], "y": [], "z": []}

    for datapoint in dataset["continuous_data"]["/imu"]:
        t_arr.append(timeStampToFloat(datapoint["header"]["stamp"]))
        imu_data["x"].append(datapoint["linear_acceleration"]["x"])
        imu_data["y"].append(datapoint["linear_acceleration"]["y"])
        imu_data["z"].append(datapoint["linear_acceleration"]["z"])

    # Reparametrize time
    t_vec_reparam = []
    for i in range(len(t_arr)):
        t_vec_reparam.append(t_arr[i] - t_arr[0])

    fig, axes = plt.subplots(1, 3)
    fig.suptitle("IMU Data w/o gravity correction")
    sns.scatterplot(x=t_vec_reparam, y=imu_data["x"], ax=axes[0])
    sns.scatterplot(x=t_vec_reparam, y=imu_data["y"], ax=axes[1])
    sns.scatterplot(x=t_vec_reparam, y=imu_data["z"], ax=axes[2])

    axes[0].set_title(r"$a_x$")
    axes[1].set_title(r"$a_y$")
    axes[2].set_title(r"$a_z$")

    plt.show()


def identifyTransitionPoints(tf_list: List, from_frame: str, to_frame: str) -> List:
    """Identify TF datapoints where there is a transition between a stationary state and movement or vice-versa."""

    # Make a copy of tf_list to remove timestamps
    tf_list_copy = deepcopy(tf_list)

    # Make a first pass through the list to remove stamps
    for i in range(len(tf_list_copy)):
        tf_list_copy[i].pop("stamp")

    transition_point_timestamp_list = []

    # Create a list of dictionaries with timestamp and distance vector from one datapoint to the next datapoint as well as the previous
    for i in range(len(tf_list_copy)):
        if (i == 0) or (i == len(tf_list_copy) - 1):
            continue

        tf_pool_t = timeStampToFloat(tf_list[i]["stamp"])

        tf_current = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_list_copy[i]
        )

        tf_previous = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_list_copy[i - 1]
        )
        tf_next = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_list_copy[i + 1]
        )

        tvec_current, quat_current = matrixToTranslationQuaternion(tf_current)
        tvec_previous, quat_previous = matrixToTranslationQuaternion(tf_previous)
        tvec_next, quat_next = matrixToTranslationQuaternion(tf_next)

        delta_previous_to_current = {
            "trans": {
                "x": tvec_current[0] - tvec_previous[0],
                "y": tvec_current[1] - tvec_previous[1],
                "z": tvec_current[2] - tvec_previous[2],
            },
            "quat": {
                "w": quat_current[0] - quat_previous[0],
                "x": quat_current[1] - quat_previous[1],
                "y": quat_current[2] - quat_previous[2],
                "z": quat_current[3] - quat_previous[3],
            },
        }

        delta_current_to_next = {
            "trans": {
                "x": tvec_next[0] - tvec_current[0],
                "y": tvec_next[1] - tvec_current[1],
                "z": tvec_next[2] - tvec_current[2],
            },
            "quat": {
                "w": quat_next[0] - quat_current[0],
                "x": quat_next[1] - quat_current[1],
                "y": quat_next[2] - quat_current[2],
                "z": quat_next[3] - quat_current[3],
            },
        }

        # Get the difference between the slopes
        delta_previous_to_next = {
            "trans": np.linalg.norm(
                [
                    delta_current_to_next["trans"][axis]
                    - delta_previous_to_current["trans"][axis]
                    for axis in ["x", "y", "z"]
                ]
            ),
            "quat": np.linalg.norm(
                [
                    delta_current_to_next["quat"][var]
                    - delta_previous_to_current["quat"][var]
                    for var in ["w", "x", "y", "z"]
                ]
            ),
        }

        if (
            delta_previous_to_next["trans"] > 0.0005
            or delta_previous_to_next["quat"] > 0.001
        ):
            transition_point_timestamp_list.append(tf_pool_t)

    return transition_point_timestamp_list


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-m",
        "--mode",
        type=str,
        default="collections",
        help="Choose whether to plot out the errors align the entire dataset or only calculate the errors at each collection. Accepted modes are: ['dataset', 'collections']",
    )
    ap.add_argument(
        "-json",
        "--json_file",
        type=str,
        required=True,
        help="Json file containing input dataset.",
    )
    ap.add_argument(
        "-sdp",
        "--save_derivation_plot",
        help="Store the results in a plot when deriving the entire dataset",
        action="store_true",
        default=False,
    )

    args = vars(ap.parse_args())

    with open(args["json_file"]) as f:
        input_dataset = json.load(f)

    # Add a grid in the background of the graphs
    sns.set_theme(style="whitegrid")

    plotIMUData(input_dataset)

    # transition_point_list = identifyTransitionPoints(
    #     tf_list=tf_lst, from_frame="world", to_frame="imu_link"
    # )

    lin_accel_dict = deriveTranslation(dataset=input_dataset, visualization=True)
    ang_vel_dict = deriveRotation(dataset=input_dataset, visualization=False)

    e = calculateErrorsAllDataPoints(
        dataset=input_dataset,
        lin_accel_dict=lin_accel_dict,
        ang_vel_dict=ang_vel_dict,
        sensor_topic="/imu",
        from_frame="world",
        to_frame="imu_link",
        save_derivation_plot=False,
    )
