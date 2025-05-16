#!/usr/bin/env python3

"""
Utilities for the derivation of TF data
"""

import argparse
import json
from copy import deepcopy
from typing import Dict, List

import numpy as np
import seaborn as sns
from atom_calibration.calibration.derivation.derivation_utils import (
    centralNumericalSecondDerivative,
    getTFList,
    getTFToDeriveList,
    quatMult,
    timeStampToFloat,
)
from atom_core.atom import getTransform
from atom_core.geometry import (
    matrixToTranslationQuaternion,
)
from matplotlib import pyplot as plt
from prettytable import PrettyTable


def deriveRotation(
    tf_list: List[Dict], poly_degree: int, visualization: bool
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
) -> List[np.poly1d]:
    """Given a list of transformations, return the functions that describe the linear accelerations.

    Inputs:
        - tf_list: a list of transformation dictionaries to use for derivation;
        - poly_degree: the degree of the polynomial functions to fit the translation data to;
        - visualization: enable graph visualization.
    Outputs:
        - p_der: a list of 3 polynomial functions to describe the linear acceleration related to each axis of translation.
    """

    if tf_list == []:
        return None, None

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

    return lin_accel, lin_vel


def deriveFromTF(
    dataset: dict,
    t: float,
    from_frame: str,
    to_frame: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
    transition_point_list: List,
) -> List[float]:
    """Given a dataset and a timestamp t, return the results of derivation for that instant of time."""

    # Don't do anything if t is a transition point
    for transition_point in transition_point_list:
        if t > transition_point + 0.5 and t < transition_point + 0.5:
            return None, None, None

    tf_lst = getTFList(dataset)

    # Get a list of the n temporally closest (wrt t) tfs to use for derivation
    tf_to_derive_lst = getTFToDeriveList(
        tf_list=tf_lst,
        from_frame=from_frame,
        to_frame=to_frame,
        t=t,
        n=neighbourhood_size,
        transition_point_list=transition_point_list,
    )

    # Sort tf_to_derive_list according to timestamp
    tf_to_derive_lst = sorted(
        tf_to_derive_lst, key=lambda x: timeStampToFloat(x["stamp"])
    )

    lin_accel_funcs, lin_vel_funcs = deriveTranslation(
        tf_list=tf_to_derive_lst, poly_degree=poly_degree, visualization=visualization
    )

    if lin_accel_funcs is None:
        return None, None, None

    lin_accel = []
    for i in range(3):
        tmp_f = lin_accel_funcs[i]
        lin_accel.append(tmp_f(t))

    lin_vel = []
    for i in range(3):
        tmp_f = lin_vel_funcs[i]
        lin_vel.append(tmp_f(t))

    ang_vel_funcs = deriveRotation(
        tf_to_derive_lst, poly_degree=poly_degree, visualization=visualization
    )

    ang_vel = []
    for i in range(3):
        tmp_f = ang_vel_funcs[i]
        ang_vel.append(tmp_f(t))

    return lin_accel, lin_vel, ang_vel


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
    sensor_name: str,
    sensor_topic: str,
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
    count = 0

    for datapoint in dataset["continuous_data"]["/tf"]:
        t = timeStampToFloat(datapoint["transforms"][0]["header"]["stamp"])
        # Derive at each timestamp

        print(count)
        count += 1

        # print(len(deriveFromTF(
        # dataset=dataset,
        # from_frame=from_frame,
        # to_frame=to_frame,
        # t=t,
        # neighbourhood_size=neighbourhood_size,
        # poly_degree=poly_degree,
        # visualization=visualization,
        # transition_point_list=transition_point_list,
        # )))

        lin_accel, lin_vel, ang_vel = deriveFromTF(
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

        derivation_results[str(t)] = {
            "lin_accel": lin_accel,
            "lin_vel": lin_vel,
            "ang_vel": ang_vel,
        }

    plt.show()

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
    transition_point_list: List,
    save_derivation_plot: bool,
) -> dict:
    """Calculate the errors in the derivation at each tf message timestamp by comparing the derivation results to the closest IMU datapoint. Plot them out."""

    # Error dict with errors vectors for each axis
    e = {
        "e_lin_accel": {"x": [], "y": [], "z": []},
        "e_ang_vel": {"x": [], "y": [], "z": []},
    }

    # Dict with tf data for plotting
    tf_data_dict = {
        "trans": {"x": [], "y": [], "z": []},
        "quat": {"x": [], "y": [], "z": [], "w": []},
    }
    # Dict with linear velocity data for plotting
    lin_vel_data_dict = {
        "x": [],
        "y": [],
        "z": [],
    }

    # Time vector
    t_vec = []

    for tf_pool in tf_list:

        # Find the closest IMU datapoint
        tf_pool_t = timeStampToFloat(tf_pool["stamp"])

        if tf_pool_t in transition_point_list:
            continue

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
        tf_pool_stamp = tf_pool.pop("stamp")  # Remove stamp so getTransform() works

        world_imu_tf = getTransform(
            from_frame="world", to_frame="imu_link", transforms=tf_pool
        )

        R = world_imu_tf[:3, :3]

        imu_accel = R @ imu_accel

        # Remove gravity
        imu_accel[2] -= 9.81

        # Calculate errors
        e["e_lin_accel"]["x"].append(
            imu_accel[0] - results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][0]
        )
        e["e_lin_accel"]["y"].append(
            imu_accel[1] - results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][1]
        )
        e["e_lin_accel"]["z"].append(
            imu_accel[2] - results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][2]
        )
        e["e_ang_vel"]["x"].append(
            imu_ang_vel[0] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][0]
        )
        e["e_ang_vel"]["y"].append(
            imu_ang_vel[1] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][1]
        )
        e["e_ang_vel"]["z"].append(
            imu_ang_vel[2] - results[str(timeStampToFloat(tf_pool_stamp))]["ang_vel"][2]
        )
        # For plotting
        t_vec.append(tf_pool_t)

        tf_trans, tf_quat = matrixToTranslationQuaternion(world_imu_tf)
        tf_data_dict["trans"]["x"].append(tf_trans[0][0])
        tf_data_dict["trans"]["y"].append(tf_trans[1][0])
        tf_data_dict["trans"]["z"].append(tf_trans[2][0])

        lin_vel_data_dict["x"].append(
            results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][0]
        )
        lin_vel_data_dict["y"].append(
            results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][1]
        )
        lin_vel_data_dict["z"].append(
            results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][2]
        )
        # NOTE: Should I add (and plot out) the rotation? I don't know that it would be super clear, due to the fact that the orientation is expressed in quaternions

    # Reparametrize time
    t_vec_reparam = []
    for i in range(len(t_vec)):
        t_vec_reparam.append(t_vec[i] - t_vec[0])

    fig1, ax1 = plt.subplots()
    sns.scatterplot(
        x=t_vec_reparam,
        y=e["e_lin_accel"]["x"],
        marker="o",
        color="red",
        s=30,
        label=r"$E_{a_x}$",
        ax=ax1,
    )
    sns.scatterplot(
        x=t_vec_reparam,
        y=e["e_lin_accel"]["y"],
        marker="o",
        color="green",
        s=30,
        label=r"$E_{a_y}$",
        ax=ax1,
    )
    sns.scatterplot(
        x=t_vec_reparam,
        y=e["e_lin_accel"]["z"],
        marker="o",
        color="blue",
        s=30,
        label=r"$E_{a_z}$",
        ax=ax1,
    )

    fig2, ax2 = plt.subplots()
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["x"], s=30, label="x", ax=ax2)
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["y"], s=30, label="y", ax=ax2)
    sns.scatterplot(x=t_vec_reparam, y=e["e_ang_vel"]["z"], s=30, label="z", ax=ax2)

    plot_titles = [
        r"$E_{a}(t)$",
        r"$E_{\omega}(t)$",
    ]

    for ax, title in zip([ax1, ax2], plot_titles):
        ax.set_title(title)

    ax1.set(xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[ms^{-2}]$")
    ax2.set(xlabel="Time since first datapoint, $t$ $[s]$", ylabel="Error $[rad/s]$")

    fig1.tight_layout()
    fig2.tight_layout()

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


def inspectDerivatives(
    dataset: Dict,
    neighbourhood_size: int,
    poly_degree: int,
    transition_point_list: List,
    tf_list: List,
    from_frame: str,
    to_frame: str,
) -> None:

    data_dict = {
        "t": [],
        "position": {"x": [], "y": [], "z": []},
    }

    # Copy it so the original keeps the timestamps
    tf_list_copy = deepcopy(tf_list)

    for tf_pool in tf_list_copy:

        # Compensate for world-imu tf
        tf_pool_stamp = tf_pool.pop("stamp")  # Remove stamp so getTransform() works

        tf_pool_t = timeStampToFloat(tf_pool_stamp)

        world_imu_tf = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_pool
        )

        # For plotting
        data_dict["t"].append(tf_pool_t)
        # Reparametrize t
        data_dict["t_reparam"] = [t - data_dict["t"][0] for t in data_dict["t"]]

        tf_trans, tf_quat = matrixToTranslationQuaternion(world_imu_tf)

        # Position Data
        data_dict["position"]["x"].append(tf_trans[0][0])
        data_dict["position"]["y"].append(tf_trans[1][0])
        data_dict["position"]["z"].append(tf_trans[2][0])

    fig, ax = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"], y=data_dict["position"]["x"], label="x", color="r"
    )
    sns.scatterplot(
        x=data_dict["t_reparam"], y=data_dict["position"]["y"], label="y", color="g"
    )
    sns.scatterplot(
        x=data_dict["t_reparam"], y=data_dict["position"]["z"], label="z", color="b"
    )

    # Create a list to append to so I can access the variable outside the function
    selected_t_lst = []

    def on_click_choose_closest_point(event):
        """Utility for picking a point in a graph. Used for inspecting derivative functions from tf_derivator in inspect mode."""

        if event.inaxes == ax:

            # Get click location
            click_x, click_y = event.xdata, event.ydata

            # Find closest point
            distances = np.abs(np.array(data_dict["t_reparam"]) - click_x)
            idx = np.argmin(distances)
            closest_x = data_dict["t_reparam"][idx]
            print(f"Closest point: ({closest_x})")
            selected_t_lst.append(closest_x)

            t_to_inspect = selected_t_lst[-1] + data_dict["t"][0]

            # Draw the velocity curves
            tf_to_derive_lst = getTFToDeriveList(
                tf_list=tf_lst,
                from_frame=from_frame,
                to_frame=to_frame,
                t=t_to_inspect,
                n=neighbourhood_size,
                transition_point_list=transition_point_list,
            )
            # Sort tf_to_derive_list according to timestamp
            tf_to_derive_lst = sorted(
                tf_to_derive_lst, key=lambda x: timeStampToFloat(x["stamp"])
            )
            lin_accel_funcs, lin_vel_funcs = deriveTranslation(
                tf_list=tf_to_derive_lst, poly_degree=poly_degree, visualization=False
            )
            t_func_start = timeStampToFloat(tf_to_derive_lst[0]["stamp"])
            t_func_end = timeStampToFloat(tf_to_derive_lst[-1]["stamp"])
            t_func = np.linspace(t_func_start, t_func_end, 1000)
            t_func_reparam = [t - data_dict["t"][0] for t in t_func]

            # Remove previous plots
            lines_plotted = [obj for obj in ax.get_lines()]
            for line in lines_plotted:
                line.remove()
            scatters = [
                obj
                for obj in ax.collections
                if isinstance(obj, plt.matplotlib.collections.PathCollection)
            ]
            scatters_to_delete = [
                obj for obj in scatters if len(obj.get_offsets()) == 1
            ]
            for scatter in scatters_to_delete:
                scatter.remove()

            line_colors = ["r", "g", "b"]
            for i in range(3):
                vel_func = lin_vel_funcs[i](t_func)
                accel_func = lin_accel_funcs[i](t_func)
                p_vel = sns.lineplot(
                    x=t_func_reparam,
                    y=vel_func,
                    ax=ax,
                    linestyle="--",
                    color=line_colors[i],
                )
                p_accel = sns.lineplot(
                    x=t_func_reparam,
                    y=accel_func,
                    ax=ax,
                    linestyle="-.",
                    color=line_colors[i],
                )

            # Now also get numerical derivative
            # Get t_i-1, t_i and t_i+1
            tmp_idx_t = data_dict["t"].index(t_to_inspect)
            data_to_derive = {
                "t": data_dict["t"][tmp_idx_t - 1 : tmp_idx_t + 2],
                "x": data_dict["position"]["x"][tmp_idx_t - 1 : tmp_idx_t + 2],
                "y": data_dict["position"]["y"][tmp_idx_t - 1 : tmp_idx_t + 2],
                "z": data_dict["position"]["z"][tmp_idx_t - 1 : tmp_idx_t + 2],
            }
            dddata_dtt = centralNumericalSecondDerivative(data_to_derive)

            for i in range(3):
                p_num_accel = sns.scatterplot(
                    x=[t_to_inspect - data_dict["t"][0]],
                    y=[dddata_dtt[i]],
                    ax=ax,
                    color=line_colors[i],
                )

            plt.draw()

    cid = fig.canvas.mpl_connect("button_press_event", on_click_choose_closest_point)

    fig.tight_layout()
    plt.show()


def plotDerivationResults(
    dataset: Dict,
    tf_list: List,
    derivation_results: Dict,
    from_frame: str,
    to_frame: str,
) -> None:
    data_dict = {
        "t": [],
        "t_reparam": [],
        "position": {"x": [], "y": [], "z": []},
        "lin_vel": {"x": [], "y": [], "z": []},
        "lin_accel": {"x": [], "y": [], "z": []},
        "lin_accel_imu": {"x": [], "y": [], "z": []},
    }

    # Copy it so the original keeps the timestamps
    tf_list_copy = deepcopy(tf_list)

    for tf_pool in tf_list_copy:

        # Compensate for world-imu tf
        tf_pool_stamp = tf_pool.pop("stamp")  # Remove stamp so getTransform() works

        tf_pool_t = timeStampToFloat(tf_pool_stamp)

        world_imu_tf = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=tf_pool
        )

        # Get closest acceleration data
        closest_imu_datapoint = min(
            dataset["continuous_data"]["/imu"],
            key=lambda datapoint: abs(
                timeStampToFloat(datapoint["header"]["stamp"]) - tf_pool_t
            ),
        )

        imu_accel = [
            closest_imu_datapoint["linear_acceleration"]["x"],
            closest_imu_datapoint["linear_acceleration"]["y"],
            closest_imu_datapoint["linear_acceleration"]["z"],
        ]

        R = world_imu_tf[:3, :3]

        imu_accel = R @ imu_accel

        imu_accel[2] -= 9.81

        # For plotting
        data_dict["t"].append(tf_pool_t)
        # Reparametrize t
        data_dict["t_reparam"].append(tf_pool_t - data_dict["t"][0])

        tf_trans, tf_quat = matrixToTranslationQuaternion(world_imu_tf)
        # Position Data
        data_dict["position"]["x"].append(tf_trans[0][0])
        data_dict["position"]["y"].append(tf_trans[1][0])
        data_dict["position"]["z"].append(tf_trans[2][0])

        # Linear Velocity Data
        data_dict["lin_vel"]["x"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][0]
        )
        data_dict["lin_vel"]["y"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][1]
        )
        data_dict["lin_vel"]["z"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_vel"][2]
        )

        # Linear Acceleration Data
        data_dict["lin_accel"]["x"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][0]
        )
        data_dict["lin_accel"]["y"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][1]
        )
        data_dict["lin_accel"]["z"].append(
            derivation_results[str(timeStampToFloat(tf_pool_stamp))]["lin_accel"][2]
        )

        data_dict["lin_accel_imu"]["x"].append(imu_accel[0])
        data_dict["lin_accel_imu"]["y"].append(imu_accel[1])
        data_dict["lin_accel_imu"]["z"].append(imu_accel[2])

    # Plot x data
    fig1, ax1 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["x"],
        marker="o",
        color="r",
        ax=ax1,
    )
    ax2 = ax1.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["x"],
        marker="s",
        color="g",
        ax=ax2,
    )
    ax3 = ax1.twinx()
    ax3.spines.right.set_position(("axes", 1.1))
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["x"],
        marker="D",
        color="b",
        alpha=0.8,
        ax=ax3,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel_imu"]["x"],
        marker="*",
        color="orange",
        ax=ax3,
    )

    # Plot y data
    fig2, ax4 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["y"],
        marker="o",
        color="r",
        ax=ax4,
    )
    ax5 = ax4.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["y"],
        marker="s",
        color="g",
        ax=ax5,
    )
    ax6 = ax4.twinx()
    ax6.spines.right.set_position(("axes", 1.1))
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["y"],
        marker="D",
        color="b",
        alpha=0.8,
        ax=ax6,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel_imu"]["y"],
        marker="*",
        color="orange",
        ax=ax6,
    )

    # Plot z data
    fig3, ax7 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["z"],
        marker="o",
        color="r",
        ax=ax7,
    )
    ax8 = ax7.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["z"],
        marker="s",
        color="g",
        ax=ax8,
    )
    ax9 = ax7.twinx()
    ax9.spines.right.set_position(("axes", 1.1))
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["z"],
        marker="D",
        alpha=0.8,
        color="b",
        ax=ax9,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel_imu"]["z"],
        marker="*",
        color="orange",
        ax=ax9,
    )

    # Some plot formatting
    ax1.set_title(r"Translation Data ($x$)")
    ax4.set_title(r"Translation Data ($y$)")
    ax7.set_title(r"Translation Data ($z$)")
    for ax in [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9]:
        ax.set(xlabel=r"Time since first datapoint, $t$ $[s]$")
    for ax in [ax1, ax4, ax7]:
        ax.set(ylabel=r"Position $[m]$")
        ax.set_ylim(-2, 2)
        ax.yaxis.label.set_color("r")
    for ax in [ax2, ax5, ax8]:
        ax.set(ylabel=r"Velocity $[m/s]$")
        ax.set_ylim(-2, 2)
        ax.yaxis.label.set_color("g")
    for ax in [ax3, ax6, ax9]:
        ax.set(ylabel=r"Acceleration $[m/s^2]$")
        ax.set_ylim(-1, 1)
        ax.yaxis.label.set_color("b")
    for fig in [fig1, fig2, fig3]:
        fig.tight_layout()

    plt.show()


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

    with open(args["json_file"]) as f:
        input_dataset = json.load(f)
    neighbourhood_size = args["neighbourhood_size"]

    tf_lst = getTFList(input_dataset)

    # Add a grid in the background of the graphs
    sns.set_theme(style="whitegrid")

    # plotIMUData(input_dataset)

    # transition_point_list = identifyTransitionPoints(
    # tf_list=tf_lst, from_frame="world", to_frame="imu_link"
    # )

    transition_point_list = []

    if args["mode"] == "collections":
        derivation_results = deriveDatasetAtCollections(
            dataset=input_dataset,
            from_frame="world",
            to_frame="imu_link",
            sensor_name="imu_hand",
            neighbourhood_size=neighbourhood_size,
            poly_degree=2,
            visualization=False,
            transition_point_list=transition_point_list,
        )

        # Calculate errors
        e = calculateErrorsAtCollections(
            dataset=input_dataset,
            results=derivation_results,
            from_frame="world",
            to_frame="imu_link",
            sensor_name="imu_hand",
        )

        # Print error table
        e_table = PrettyTable()
        e_table.field_names = [
            "Collection",
            "t",
            "E_lin_accel (m/s^2)",
            "E_ang_vel (rad/s)",
        ]

        e_table.add_rows(
            [
                [
                    collection_key,
                    round(
                        timeStampToFloat(
                            input_dataset["collections"][collection_key]["data"][
                                "imu_hand"
                            ]["header"]["stamp"]
                        )
                        - timeStampToFloat(
                            input_dataset["continuous_data"]["/imu"][0]["header"][
                                "stamp"
                            ],
                        ),
                        4,
                    ),
                    round(float(e[collection_key]["e_lin_accel"]), 4),
                    round(float(e[collection_key]["e_ang_vel"]), 4),
                ]
                for collection_key in e.keys()
            ]
        )

        print(e_table)

    if args["mode"] == "dataset":
        derivation_results = deriveDatasetAllDataPoints(
            dataset=input_dataset,
            from_frame="world",
            to_frame="imu_link",
            sensor_name="imu_hand",
            sensor_topic="/imu",
            neighbourhood_size=neighbourhood_size,
            poly_degree=args["poly_degree"],
            visualization=False,
            transition_point_list=transition_point_list,
        )

        plotDerivationResults(
            dataset=input_dataset,
            tf_list=tf_lst,
            derivation_results=derivation_results,
            from_frame="world",
            to_frame="imu_link",
        )

        e = calculateErrorsAllDataPoints(
            dataset=input_dataset,
            tf_list=tf_lst,
            results=derivation_results,
            sensor_topic="/imu",
            from_frame="world",
            to_frame="imu_link",
            transition_point_list=transition_point_list,
            save_derivation_plot=args["save_derivation_plot"],
        )

    if args["mode"] == "inspect":
        # The idea is to plot out the derivatives in a given point
        inspectDerivatives(
            dataset=input_dataset,
            neighbourhood_size=neighbourhood_size,
            poly_degree=args["poly_degree"],
            transition_point_list=transition_point_list,
            tf_list=tf_lst,
            from_frame="world",
            to_frame="imu_link",
        )
