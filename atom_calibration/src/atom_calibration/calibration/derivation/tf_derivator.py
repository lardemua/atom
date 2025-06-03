#!/usr/bin/env python3

import argparse
import json
from copy import deepcopy
from typing import Any, Dict, List, Tuple

from atom_core.utilities import atomError
import numpy as np
import seaborn as sns
from atom_calibration.calibration.derivation.derivation_utils import (
    getTFList,
    plotDerivationResults,
    plotIMUData,
    timeStampToFloat,
)
from atom_core.atom import getTransform
from atom_core.geometry import (
    matrixToTranslationQuaternion,
    translationQuaternionToTransform,
)
from matplotlib import pyplot as plt
from scipy.signal import savgol_filter
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

    R_arr = []

    # For each datapoint
    for i in range(len(tf_data_dict["t"])):
        quat = [tf_data_dict["quat"][var][i] for var in ["x", "y", "z", "w"]]
        tvec = [tf_data_dict["trans"][var][i] for var in ["x", "y", "z"]]

        # get tf matrix
        M = translationQuaternionToTransform(tvec, quat)

        R_arr.append(M[:3, :3])

    R_arr = np.array(R_arr)
    dR_arr = np.zeros_like(R_arr)

    # Derive each element of rotation matrix
    for j in range(3):
        for k in range(3):
            series = R_arr[:, j, k]
            deriv = savgol_filter(
                x=series,
                window_length=neighbourhood_size,
                polyorder=poly_degree,
                deriv=1,
                delta=dt,
            )
            dR_arr[:, j, k] = deriv

    # Compute ang_vels
    ang_vels = {"x": [], "y": [], "z": []}
    for k in range(R_arr.shape[0]):
        R = R_arr[k]
        dR = dR_arr[k]
        omega_hat = R.T @ dR

        ang_vels["x"].append(omega_hat[2, 1])
        ang_vels["y"].append(omega_hat[0, 2])
        ang_vels["z"].append(omega_hat[1, 0])

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
    lin_vel_arr: dict[str, Any] = {"x": lin_vel_x, "y": lin_vel_y, "z": lin_vel_z}
    lin_accel_arr: dict[str, Any] = {
        "x": lin_accel_x,
        "y": lin_accel_y,
        "z": lin_accel_z,
    }

    return lin_vel_arr, lin_accel_arr


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
    noise: tuple,
) -> dict:
    """
    Derive for all timestamps corresponding to collections in a dataset.
    Return a dictionary containing the derivation results for each collection.
    """

    # Get list of timestamps to integrate for
    derivation_results = {}

    tf_pool_lst: List[Dict[Any, Any]] = getTFList(dataset=dataset)
    tf_pool_lst_copy = deepcopy(tf_pool_lst)

    # with open("test.json", "w") as f:
    #     json.dump(tf_pool_lst_copy, f, indent=4)

    # Organize the data in lists for plotting and deriving
    data_dict = {
        "t": [],
        "trans": {"x": [], "y": [], "z": []},
        "quat": {"x": [], "y": [], "z": [], "w": []},
    }

    noise_trans = noise[0]
    noise_rot = noise[1]

    for tf_pool_idx in range(len(tf_pool_lst_copy)):
    # for tf_pool in tf_pool_lst_copy:
        tf_pool = tf_pool_lst_copy[tf_pool_idx]
        data_dict["t"].append(timeStampToFloat(stamp=tf_pool.pop("stamp")))
        
        # Calculate the new atomic tf with noise.
        if tf_pool_idx == 0:
            # Add noise to the atomic tf of the imu
            for tf_key, transform in tf_pool.items():
                if transform["child"] == to_frame:
                    # Check if the transform is fixed. It should be, given how ATOM works, but it's better to check regardless.
                    if dataset["transforms"][f"{transform['parent']}-{transform['child']}"]["type"] != "fixed":
                        atomError("The TF you're trying to add noise to isn't fixed! Are you sure your dataset was correctly collected?")

                    quat = transform["quat"]
                    trans = transform["trans"]

                    v = np.random.uniform(-1.0, 1.0, 3)
                    v = v / np.linalg.norm(v)
                    new_trans = trans + v * noise_trans

                    v = np.random.choice([-1.0, 1.0], 3) * noise_rot
                    euler_angles = tf.transformations.euler_from_quaternion(quat)
                    new_angles = euler_angles + v
                    new_quat = tf.transformations.quaternion_from_euler(
                        new_angles[0], new_angles[1], new_angles[2]
                    )

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

    lin_vel_arr, lin_accel_arr = deriveTranslation(
        tf_data_dict=data_dict,
        poly_degree=poly_degree,
        neighbourhood_size=neighbourhood_size,
    )

    ang_vels = deriveRotation(
        tf_data_dict=data_dict,
        poly_degree=poly_degree,
        neighbourhood_size=neighbourhood_size,
    )

    derivation_results = {
        "lin_accel": lin_accel_arr,
        "lin_vel": lin_vel_arr,
        "ang_vel": ang_vels,
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
    ap.add_argument(
        "-nig",
        "--noisy_initial_guess",
        nargs=2,
        metavar=("translation", "rotation"),
        help="Magnitude of noise to add to the initial guess atomic transformations set before starting optimization [meters, radians].",
        type=float,
        default=[0.0, 0.0],
    )
    ap.add_argument("-ss", "--sample_seed", help="Sampling seed", type=int)

    args = vars(ap.parse_args())
    neighbourhood_size = args["neighbourhood_size"]

    with open(args["json_file"]) as f:
        input_dataset = json.load(f)

    # Add a grid in the background of the graphs
    sns.set_theme(style="whitegrid")

    plotIMUData(dataset=input_dataset)

    dataset_ground_truth = deepcopy(x=input_dataset)

    selected_collection_key = list(input_dataset["collections"].keys())[0]

    tf_lst = getTFList(input_dataset)

    derivation_results = deriveDatasetAllDataPoints(
        dataset=input_dataset,
        from_frame="world",
        to_frame="accelerometer",
        neighbourhood_size=neighbourhood_size,
        poly_degree=args["poly_degree"],
        noise=args["noisy_initial_guess"],
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
