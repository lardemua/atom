#!/usr/bin/env python3

"""
Utilities for the derivation of TF data
"""

import argparse
import json
from copy import deepcopy
from typing import Any, Dict, List, Tuple

import numpy as np
import seaborn as sns
from atom_calibration.calibration.derivation.derivation_utils import (
    centralNumericalSecondDerivative,
    getTFList,
    getTFToDeriveList,
    inspectDerivatives,
    plotDerivationResults,
    plotIMUData,
    quatMult,
    timeStampToFloat,
)
from atom_core.atom import getTransform
from atom_core.geometry import (
    matrixToTranslationQuaternion,
)
from matplotlib import pyplot as plt
from numpy import float64, generic
from numpy._typing._generic_alias import NDArray
from pandas.io.formats.info import frame_see_also_sub
from prettytable import PrettyTable
from scipy.signal import savgol_filter


def deriveRotation(
    tf_list: List[Dict[str, Any]], poly_degree: int, visualization: bool
) -> List[np.poly1d]:
    """Given a list of transformations, return the functions that describe the angular velocities.
    Angular velocities are calculated using the following:

        dw = 2 * quaternionMultiplication(dq, q), with dw = [0, wx, wy, wz]

    Inputs:
        - tf_list: a list of transformation dictionaries to use for derivation;
        - poly_degree: the degree of the polynomial functions to fit the rotation data to;
        - visualization: enable graph visualization.
    Outputs:
        - p_der: a list of 3 polynomial functions to describe the angular velocity related to each axis of rotation.
    """
    # Get time values
    t_arr = np.array([timeStampToFloat(tf["stamp"]) for tf in tf_list])

    # for each rotation variable
    rot_array = np.array(
        [
            [tf["quat"][0] for tf in tf_list],
            [tf["quat"][1] for tf in tf_list],
            [tf["quat"][2] for tf in tf_list],
            [tf["quat"][3] for tf in tf_list],
        ]
    )

    q = [np.polyfit(t_arr, rot_array[i], deg=poly_degree) for i in range(4)]
    dq = []

    if visualization:
        fig, axes = plt.subplots(2, 4)

    # Get quaternion derivatives, dq
    for i in range(4):
        poly_func = np.poly1d(q[i])

        dq.append(np.polyder(poly_func))

        if visualization:
            sns.scatterplot(x=t_arr, y=rot_array[i], ax=axes[0, i])
            x_func = np.linspace(t_arr.min(), t_arr.max(), 1000)
            y_func = poly_func(x_func)
            sns.lineplot(x=x_func, y=y_func, color="red", ax=axes[0, i])

            yder_func = dq[i](x_func)
            sns.lineplot(x=x_func, y=yder_func, color="green", ax=axes[1, i])

    q_conjugate = [
        np.poly1d(q[0]),
        np.poly1d(-q[1]),
        np.poly1d(-q[2]),
        np.poly1d(-q[3]),
    ]

    omega = quatMult(2 * dq, q_conjugate)

    if visualization:
        plt.show()

    ang_vels = [omega[1], omega[2], omega[3]]

    return ang_vels


def deriveTranslation(
    tf_list: List[Dict], poly_degree: int, visualization: bool
) -> Tuple:
    """Given a list of transformations, return the functions that describe the linear accelerations.

    Inputs:
        - tf_list: a list of transformation dictionaries to use for derivation;
        - poly_degree: the degree of the polynomial functions to fit the translation data to;
        - visualization: enable graph visualization.
    Outputs:
        - p_der: a list of 3 polynomial functions to describe the linear acceleration related to each axis of translation.
    """

    if tf_list == []:
        return None, None, None

    # Get time values
    t_arr = np.array([timeStampToFloat(tf["stamp"]) for tf in tf_list])

    # for each translation variable
    trans_array = np.array(
        [
            [tf["trans"][0] for tf in tf_list],
            [tf["trans"][1] for tf in tf_list],
            [tf["trans"][2] for tf in tf_list],
        ]
    )

    q = []
    for i in range(3):
        coeffs = np.polyfit(t_arr, trans_array[i], deg=poly_degree)
        q.append(coeffs)
    q_funcs = []
    dq = []
    ddq = []

    if visualization:
        fig, axes = plt.subplots(3, 3)

        plot_titles = [
            r"$x(t)$",
            r"$y(t)$",
            r"$z(t)$",
            r"$\dot{x}(t)$",
            r"$\dot{y}(t)$",
            r"$\dot{z}(t)$",
            r"$\ddot{x}(t)$",
            r"$\ddot{y}(t)$",
            r"$\ddot{z}(t)$",
        ]

        for ax, title in zip(axes.reshape(-1), plot_titles):
            ax.set_title(title)

    # Get translation derivatives, dq
    for i in range(3):
        poly_func = np.poly1d(q[i])
        q_funcs.append(poly_func)

        dq.append(np.polyder(poly_func))

        dq_func = np.poly1d(dq[i])
        ddq.append(np.polyder(dq_func))

        if visualization:
            sns.scatterplot(x=t_arr, y=trans_array[i], ax=axes[0, i])
            x_func = np.linspace(t_arr.min(), t_arr.max(), 1000)
            y_func = poly_func(x_func)
            sns.lineplot(x=x_func, y=y_func, color="red", ax=axes[0, i])

            yder_func = dq[i](x_func)
            sns.lineplot(x=x_func, y=yder_func, color="green", ax=axes[1, i])

            y2der_func = ddq[i](x_func)
            sns.lineplot(x=x_func, y=y2der_func, color="blue", ax=axes[2, i])

    if visualization:
        fig.tight_layout
        plt.show()

    lin_accel = [ddq[0], ddq[1], ddq[2]]
    lin_vel = [dq[0], dq[1], dq[2]]
    pos_curve = [q_funcs[0], q_funcs[1], q_funcs[2]]

    return lin_accel, lin_vel, pos_curve


def deriveDatasetAtCollections(
    dataset: dict,
    from_frame: str,
    to_frame: str,
    sensor_name: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
    transition_point_list: List,
) -> dict:
    """
    Derive for all timestamps corresponding to collections in a dataset.
    Return a dictionary containing the derivation results for each collection.
    """

    # Get list of timestamps to integrate for
    derivation_results = {}
    for collection_key, collection in dataset["collections"].items():
        t = timeStampToFloat(collection["data"][sensor_name]["header"]["stamp"])

        # Derive at each timestamp
        lin_accel, ang_vel = deriveFromTF(
            dataset=dataset,
            from_frame=from_frame,
            to_frame=to_frame,
            t=t,
            neighbourhood_size=neighbourhood_size,
            poly_degree=poly_degree,
            visualization=visualization,
            transition_point_list=transition_point_list,
        )

        if lin_accel is None and ang_vel is None:
            continue

        derivation_results[collection_key] = {
            "lin_accel": lin_accel,
            "ang_vel": ang_vel,
        }

    return derivation_results


def deriveDatasetAllDataPoints(
    dataset: dict,
    from_frame: str,
    to_frame: str,
    neighbourhood_size: int,
    poly_degree: int,
) -> dict:
    """
    Derive for all timestamps corresponding to collections in a dataset.
    Return a dictionary containing the derivation results for each collection.
    """

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

    for tf_pool in tf_pool_lst_copy:
        data_dict["t"].append(timeStampToFloat(stamp=tf_pool.pop("stamp")))

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

    dt = data_dict["t"][1] - data_dict["t"][0]

    # Now derive the data
    lin_vel_x = savgol_filter(
        x=data_dict["trans"]["x"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_vel_y = savgol_filter(
        x=data_dict["trans"]["y"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_vel_z = savgol_filter(
        x=data_dict["trans"]["z"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=1,
        delta=dt,
    )
    lin_accel_x = savgol_filter(
        x=data_dict["trans"]["x"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_accel_y = savgol_filter(
        x=data_dict["trans"]["y"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_accel_z = savgol_filter(
        x=data_dict["trans"]["z"],
        window_length=neighbourhood_size,
        polyorder=poly_degree,
        deriv=2,
        delta=dt,
    )
    lin_vel: dict[str, Any] = {"x": lin_vel_x, "y": lin_vel_y, "z": lin_vel_z}
    lin_accel: dict[str, Any] = {"x": lin_accel_x, "y": lin_accel_y, "z": lin_accel_z}

    derivation_results = {"lin_accel": lin_accel, "lin_vel": lin_vel}

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
            "e_lin_accel": np.linalg.norm(np.array(imu_accel) - result["lin_accel"]),
            "e_ang_vel": np.linalg.norm(np.array(imu_ang_vel) - result["ang_vel"]),
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
        "lin_accel": {"x": [], "y": [], "z": []},
        "lin_accel_imu": {"x": [], "y": [], "z": []},
        "e_lin_accel": {"x": [], "y": [], "z": []},
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
        # imu_ang_vel = [*closest_sensor_datapoint["angular_velocity"].values()]

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
            imu_accel[0] - results["lin_accel"]["x"][i]
        )
        data_dict["e_lin_accel"]["y"].append(
            imu_accel[1] - results["lin_accel"]["y"][i]
        )
        data_dict["e_lin_accel"]["z"].append(
            imu_accel[2] - results["lin_accel"]["z"][i]
        )
        # e["e_ang_vel"]["x"].append(
        #     imu_ang_vel[0] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][0]
        # )
        # e["e_ang_vel"]["y"].append(
        #     imu_ang_vel[1] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][1]
        # )
        # e["e_ang_vel"]["z"].append(
        #     imu_ang_vel[2] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][2]
        # )

        # NOTE: Should I add (and plot out) the rotation? I don't know that it would be super clear, due to the fact that the orientation is expressed in quaternions

    fig1, ax1 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["e_lin_accel"]["x"],
        marker="o",
        color="r",
        s=30,
        label=r"$E_{a_x}$",
        alpha=0.9,
        ax=ax1,
    )
    fig2, ax2 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["e_lin_accel"]["y"],
        marker="o",
        color="g",
        s=30,
        label=r"$E_{a_y}$",
        alpha=0.9,
        ax=ax2,
    )
    fig3, ax3 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["e_lin_accel"]["z"],
        marker="o",
        color="b",
        s=30,
        label=r"$E_{a_z}$",
        alpha=0.9,
        ax=ax3,
    )

    # fig2, ax2 = plt.subplots()
    # sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["x"], s=30, label="x", ax=ax2)
    # sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["y"], s=30, label="y", ax=ax2)
    # sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["z"], s=30, label="z", ax=ax2)

    plot_titles = [
        r"$E_{a}(t)$",
        r"$E_{a}(t)$",
        r"$E_{a}(t)$",
        # r"$E_{\omega}(t)$",
    ]

    for ax, title in zip([ax1, ax2, ax3], plot_titles):
        # for ax, title in zip([ax1, ax2], plot_titles):
        ax.set_title(title)
        ax.set(
            xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[ms^{-2}]$"
        )

    # ax2.set(xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[rad/s]$")

    fig1.tight_layout()
    # fig2.tight_layout()

    plt.show()

    # Error dict returned
    e = {"lin_accel": data_dict["e_lin_accel"]}

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

    args = vars(ap.parse_args())
    neighbourhood_size = args["neighbourhood_size"]

    with open(args["json_file"]) as f:
        input_dataset = json.load(f)

    # Add a grid in the background of the graphs
    sns.set_theme(style="whitegrid")

    plotIMUData(dataset=input_dataset)

    tf_lst = getTFList(input_dataset)

    derivation_results = deriveDatasetAllDataPoints(
        dataset=input_dataset,
        from_frame="world",
        to_frame="accelerometer",
        neighbourhood_size=neighbourhood_size,
        poly_degree=args["poly_degree"],
    )

    plotDerivationResults(
        dataset=input_dataset,
        tf_list=tf_lst,
        derivation_results=derivation_results,
        from_frame="world",
        to_frame="accelerometer",
    )

    e = calculateErrorsAllDataPoints(
        dataset=input_dataset,
        tf_list=tf_lst,
        results=derivation_results,
        sensor_topic="/imu",
        from_frame="world",
        to_frame="accelerometer",
    )
