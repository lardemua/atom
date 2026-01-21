#!/usr/bin/env python3

from datetime import datetime
import os
from copy import deepcopy
from math import floor
from pathlib import Path
from typing import Dict, List

import numpy as np
import scipy
import seaborn as sns
from atom_core.atom import getTransform
from atom_core.geometry import matrixToTranslationQuaternion
from atom_core.utilities import atomError
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter


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

    for i in range(3):
        axes[i].set_ylim(-10, 10)

    plt.show()


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


def getTFList(dataset: Dict) -> List[Dict]:
    """Acquire a list of tfs from the continuous data."""

    # Check if dataset has /tf and /tf_static continuous data collected
    if (
        "/tf" not in dataset["continuous_data"].keys()
        or "/tf_static" not in dataset["continuous_data"].keys()
    ):
        atomError("Dataset does not contain /tf and /tf_static continuous data!")

    tf_list = []

    # Create transforms list of dict with data from /tf and /tf_static
    # tmp_i = 0
    for tf_msg in dataset["continuous_data"]["/tf"]:

        if len(tf_msg["transforms"]) == 0:
            continue

        if len(tf_msg["transforms"]) == 0:
            continue

        tf_dict_to_append = {}

        # Get stamp from one of the transforms in the tf_msg
        # NOTE: Since all of the TFs in the same "transforms" field have the same stamp, I can just access the first one
        # print(tmp_i)
        # tmp_i += 1
        tf_dict_to_append["stamp"] = tf_msg["transforms"][0]["header"]["stamp"]

        for tf in tf_msg["transforms"]:
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

            tf_dict_to_append[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": quat,
                "trans": trans,
            }

        # Include transforms from /tf_static. Only consider the last message.
        chosen_idx = 0
        for idx in range(len(dataset["continuous_data"]["/tf_static"])):
            if dataset["continuous_data"]["/tf_static"][idx]["transforms"] != []:
                chosen_idx = idx
                break

        for tf in dataset["continuous_data"]["/tf_static"][chosen_idx]["transforms"]:
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

            tf_dict_to_append[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": quat,
                "trans": trans,
            }

        tf_list.append(tf_dict_to_append)

    return tf_list


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

        tf_pool_prev_t = timeStampToFloat(tf_list[i - 1]["stamp"])
        tf_pool_t = timeStampToFloat(tf_list[i]["stamp"])
        tf_pool_next_t = timeStampToFloat(tf_list[i + 1]["stamp"])

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
            delta_previous_to_next["trans"] > 0.0001
            or delta_previous_to_next["quat"] > 0.001
        ):
            transition_point_timestamp_list.append(tf_pool_t)
            # transition_point_timestamp_list.append(tf_pool_prev_t)
            # transition_point_timestamp_list.append(tf_pool_next_t)

    return transition_point_timestamp_list


def plotTFs(tf_list: List) -> None:

    plt.figure()

    t_vec = []
    x_vec = []
    y_vec = []
    z_vec = []

    tf_pool_t_0 = timeStampToFloat(tf_list[0]["stamp"])
    for tf_pool in tf_list:

        tf_pool_t = timeStampToFloat(tf_pool.pop("stamp"))
        t_vec.append(tf_pool_t - tf_pool_t_0)

        world_imu_tf = getTransform(
            from_frame="world", to_frame="imu_link", transforms=tf_pool
        )

        x_vec.append(world_imu_tf[0, 3])
        y_vec.append(world_imu_tf[1, 3])
        z_vec.append(world_imu_tf[2, 3])

    sns.scatterplot(x=t_vec, y=x_vec, label="x")
    sns.scatterplot(x=t_vec, y=y_vec, label="y")
    sns.scatterplot(x=t_vec, y=z_vec, label="z")

    plt.show()


def getTFToDeriveList(
    tf_list: List,
    from_frame: str,
    to_frame: str,
    t: float,
    n: int,
) -> List[Dict]:
    """Get a list of n source-target TFs in the temporal neighbourhood of timestamp."""

    # Find the n tf dicts (full topological tree) closest to the timestamp
    closest_tf_dicts = []

    tf_list_copy = deepcopy(tf_list)
    for i in range(n):
        min_t_dist = None
        min_element = None
        for element in tf_list_copy:
            element_t = timeStampToFloat(element["stamp"])
            t_dist = abs(t - element_t)

            # if min_element is None or (element_t <= t and t_dist < min_t_dist):
            if min_element is None or t_dist < min_t_dist:
                min_t_dist = t_dist
                min_element = element

        closest_tf_dicts.append(min_element)
        tf_list_copy.remove(min_element)

    # Get the source-target tfs
    source_to_target_tf_lst = []
    for element in closest_tf_dicts:
        stamp = element.pop("stamp")
        source_target_tf = getTransform(
            from_frame=from_frame, to_frame=to_frame, transforms=element
        )

        tvec, quat = matrixToTranslationQuaternion(source_target_tf)

        # tvec need to be turned into a row instead of a column
        trans = np.array([tvec[i, 0] for i in range(3)])

        # Use dict form for tf

        dict_to_append = {
            "parent": from_frame,
            "child": to_frame,
            "trans": trans,
            "quat": quat,
            "stamp": stamp,
        }

        source_to_target_tf_lst.append(dict_to_append)

    return source_to_target_tf_lst


def convertRotationsInTFToEuler(tf_list: List[Dict]) -> List[Dict]:
    """Convert the rotations in a list of TFs to euler angles (XYZ convention)"""

    for tf in tf_list:
        r = Rotation.from_quat(tf["quat"])
        r_euler = r.as_euler(seq="XYZ")

        tf["euler"] = r_euler.tolist()

    return tf_list


def smoothImuData(dataset, args) -> None:
    """
    Applies Savitzky-Golay filter to IMU data for smoothing.
    """

    data_dict = {
        "t": [],
        "lin_accel_imu": {"x": [], "y": [], "z": []},
        "ang_vel_imu": {"x": [], "y": [], "z": []},
    }

    for sensor_key, sensor in dataset["sensors"].items():
        if sensor["modality"] != "imu":
            continue
        sensor_name = sensor_key

    sensor_topic = dataset["sensors"][sensor_name]["topic"]

    for datapoint in dataset["continuous_data"][sensor_topic]:
        data_dict["t"].append(timeStampToFloat(datapoint["header"]["stamp"]))
        data_dict["lin_accel_imu"]["x"].append(datapoint["linear_acceleration"]["x"])
        data_dict["lin_accel_imu"]["y"].append(datapoint["linear_acceleration"]["y"])
        data_dict["lin_accel_imu"]["z"].append(datapoint["linear_acceleration"]["z"])
        data_dict["ang_vel_imu"]["x"].append(datapoint["angular_velocity"]["x"])
        data_dict["ang_vel_imu"]["y"].append(datapoint["angular_velocity"]["y"])
        data_dict["ang_vel_imu"]["z"].append(datapoint["angular_velocity"]["z"])

    dt = 0
    k = 0
    while dt == 0:
        dt = data_dict["t"][k + 1] - data_dict["t"][k]
        k += 1

    lin_accel_x_smooth = savgol_filter(
        x=data_dict["lin_accel_imu"]["x"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )
    lin_accel_y_smooth = savgol_filter(
        x=data_dict["lin_accel_imu"]["y"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )
    lin_accel_z_smooth = savgol_filter(
        x=data_dict["lin_accel_imu"]["z"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )
    ang_vel_x_smooth = savgol_filter(
        x=data_dict["ang_vel_imu"]["x"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )
    ang_vel_y_smooth = savgol_filter(
        x=data_dict["ang_vel_imu"]["y"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )
    ang_vel_z_smooth = savgol_filter(
        x=data_dict["ang_vel_imu"]["z"],
        window_length=args["neighbourhood_size"],
        polyorder=args["poly_degree"],
        deriv=0,
        delta=dt,
    )

    # Find timestamps of IMU data from collections
    timestamps = []
    for collection_key, collection in dataset["collections"].items():
        timestamps.append(
            (collection_key, collection["data"][sensor_name]["header"]["stamp"])
        )

    dataset["continuous_data"][f"{sensor_topic}_original"] = []
    for i in range(len(dataset["continuous_data"][sensor_topic])):
        # Copy to "sensor_topic_original"
        dataset["continuous_data"][f"{sensor_topic}_original"].append(
            dataset["continuous_data"][sensor_topic][i]
        )
        # Replace datapoint for smoothed value in sensor_topic
        dataset["continuous_data"][sensor_topic][i]["linear_acceleration"]["x"] = (
            lin_accel_x_smooth[i]
        )
        dataset["continuous_data"][sensor_topic][i]["linear_acceleration"]["y"] = (
            lin_accel_y_smooth[i]
        )
        dataset["continuous_data"][sensor_topic][i]["linear_acceleration"]["z"] = (
            lin_accel_z_smooth[i]
        )
        dataset["continuous_data"][sensor_topic][i]["angular_velocity"]["x"] = (
            ang_vel_x_smooth[i]
        )
        dataset["continuous_data"][sensor_topic][i]["angular_velocity"]["y"] = (
            ang_vel_y_smooth[i]
        )
        dataset["continuous_data"][sensor_topic][i]["angular_velocity"]["z"] = (
            ang_vel_z_smooth[i]
        )

        # Now, if the datapoint matches any of the collections, replace the values in the collections as well
        check_list = [
            x
            for x in timestamps
            if x[1] == dataset["continuous_data"][sensor_topic][i]["header"]["stamp"]
        ]

        if check_list == []:
            continue

        collection_key = check_list[0][0]

        for axis in ["x", "y", "z"]:
            dataset["collections"][collection_key]["data"][sensor_name][
                "linear_acceleration"
            ][axis] = dataset["continuous_data"][sensor_topic][i][
                "linear_acceleration"
            ][
                axis
            ]
            dataset["collections"][collection_key]["data"][sensor_name][
                "angular_velocity"
            ][axis] = dataset["continuous_data"][sensor_topic][i]["angular_velocity"][
                axis
            ]

    return


def plotDerivationResults(
    dataset: Dict,
    tf_list: List,
    derivation_results: Dict,
    from_frame: str,
    to_frame: str,
    noise: tuple,
    dataset_name: str,
    ignore_gravity: bool,
    sensor_name: str,
    neighbourhood_size: int,
    poly_degree: int,
    gravity: float,
) -> None:

    # NOTE: It doesn't make sense to plot out orientation since it's expressed in quaternions
    data_dict = {
        "t": [],
        "t_reparam": [],
        "collection_times": [],
        "position": {"x": [], "y": [], "z": []},
        "lin_vel": {"x": [], "y": [], "z": []},
        "lin_accel": {"x": [], "y": [], "z": []},
        "lin_accel_imu": {"x": [], "y": [], "z": []},
        "lin_accel_imu_original": {"x": [], "y": [], "z": []},
        "angs": {"x": [], "y": [], "z": []},
        "ang_vel": {"x": [], "y": [], "z": []},
        "ang_vel_imu": {"x": [], "y": [], "z": []},
        "ang_vel_imu_original": {"x": [], "y": [], "z": []},
        "ang_speed": [],
        "ang_speed_imu": [],
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

        sensor_topic = dataset["sensors"][sensor_name]["topic"]
        # Get closest acceleration data
        closest_imu_datapoint = min(
            dataset["continuous_data"][sensor_topic],
            key=lambda datapoint: abs(
                timeStampToFloat(datapoint["header"]["stamp"]) - tf_pool_t
            ),
        )

        # Get data from IMU
        imu_accel = [
            closest_imu_datapoint["linear_acceleration"]["x"],
            closest_imu_datapoint["linear_acceleration"]["y"],
            closest_imu_datapoint["linear_acceleration"]["z"],
        ]

        imu_ang_vel = [
            closest_imu_datapoint["angular_velocity"]["x"],
            closest_imu_datapoint["angular_velocity"]["y"],
            closest_imu_datapoint["angular_velocity"]["z"],
        ]

        # For plotting pre-smoothing IMU data...
        closest_imu_datapoint_original = min(
            dataset["continuous_data"][f"{sensor_topic}_original"],
            key=lambda datapoint: abs(
                timeStampToFloat(datapoint["header"]["stamp"]) - tf_pool_t
            ),
        )

        # Get data from IMU
        imu_accel_original = [
            closest_imu_datapoint_original["linear_acceleration"]["x"],
            closest_imu_datapoint_original["linear_acceleration"]["y"],
            closest_imu_datapoint_original["linear_acceleration"]["z"],
        ]

        imu_ang_vel_original = [
            closest_imu_datapoint_original["angular_velocity"]["x"],
            closest_imu_datapoint_original["angular_velocity"]["y"],
            closest_imu_datapoint_original["angular_velocity"]["z"],
        ]

        # Norm of velocity vectors
        imu_ang_speed = np.linalg.norm(imu_ang_vel)

        # Convert to Rotation instance and re-convert to match to derivative
        imu_ang_vel_r = Rotation.from_rotvec(imu_ang_vel)
        imu_ang_vel = imu_ang_vel_r.as_matrix()
        imu_ang_vel_r = Rotation.from_matrix(imu_ang_vel)
        imu_ang_vel = imu_ang_vel_r.as_rotvec()

        imu_ang_vel_original_r = Rotation.from_rotvec(imu_ang_vel_original)
        imu_ang_vel_original = imu_ang_vel_r.as_matrix()
        imu_ang_vel_original_r = Rotation.from_matrix(imu_ang_vel_original)
        imu_ang_vel_original = imu_ang_vel_r.as_rotvec()

        R = world_imu_tf[:3, :3]
        imu_accel = R @ imu_accel
        imu_ang_vel = R @ imu_ang_vel
        imu_accel_original = R @ imu_accel_original
        imu_ang_vel_original = R @ imu_ang_vel_original

        if not ignore_gravity:
            imu_accel[2] -= gravity
            imu_accel_original[2] -= gravity

        # For plotting
        data_dict["t"].append(tf_pool_t)
        # Reparametrize t
        data_dict["t_reparam"].append(tf_pool_t - data_dict["t"][0])

        tf_trans, tf_quat = matrixToTranslationQuaternion(world_imu_tf)
        # Position Data
        data_dict["position"]["x"].append(tf_trans[0][0])
        data_dict["position"]["y"].append(tf_trans[1][0])
        data_dict["position"]["z"].append(tf_trans[2][0])

        data_dict["lin_accel_imu"]["x"].append(imu_accel[0])
        data_dict["lin_accel_imu"]["y"].append(imu_accel[1])
        data_dict["lin_accel_imu"]["z"].append(imu_accel[2])

        data_dict["ang_vel_imu"]["x"].append(imu_ang_vel[0])
        data_dict["ang_vel_imu"]["y"].append(imu_ang_vel[1])
        data_dict["ang_vel_imu"]["z"].append(imu_ang_vel[2])

        data_dict["lin_accel_imu_original"]["x"].append(imu_accel_original[0])
        data_dict["lin_accel_imu_original"]["y"].append(imu_accel_original[1])
        data_dict["lin_accel_imu_original"]["z"].append(imu_accel_original[2])

        data_dict["ang_vel_imu_original"]["x"].append(imu_ang_vel_original[0])
        data_dict["ang_vel_imu_original"]["y"].append(imu_ang_vel_original[1])
        data_dict["ang_vel_imu_original"]["z"].append(imu_ang_vel_original[2])

        data_dict["ang_speed_imu"].append(imu_ang_speed)

    # Linear Velocity Data
    data_dict["lin_vel"]["x"] = derivation_results["lin_vel"]["x"]
    data_dict["lin_vel"]["y"] = derivation_results["lin_vel"]["y"]
    data_dict["lin_vel"]["z"] = derivation_results["lin_vel"]["z"]

    # Linear Acceleration Data
    data_dict["lin_accel"]["x"] = derivation_results["lin_accel"]["x"]
    data_dict["lin_accel"]["y"] = derivation_results["lin_accel"]["y"]
    data_dict["lin_accel"]["z"] = derivation_results["lin_accel"]["z"]

    # Orientation Data
    data_dict["angs"]["x"] = derivation_results["angs"]["x"]
    data_dict["angs"]["y"] = derivation_results["angs"]["y"]
    data_dict["angs"]["z"] = derivation_results["angs"]["z"]

    # Angular Velocity Data
    data_dict["ang_vel"]["x"] = derivation_results["ang_vel"]["x"]
    data_dict["ang_vel"]["y"] = derivation_results["ang_vel"]["y"]
    data_dict["ang_vel"]["z"] = derivation_results["ang_vel"]["z"]

    data_dict["ang_speed"] = derivation_results["ang_speed"]

    selected_sensor = None
    for sensor_key, sensor in dataset["sensors"].items():
        if sensor["topic"] == sensor_topic:
            selected_sensor = sensor_key

    # get collection times to mark on the plots when the collections were taken
    for _, collection in dataset["collections"].items():
        collection_t = timeStampToFloat(
            stamp=collection["data"][selected_sensor]["header"]["stamp"]
        )
        data_dict["collection_times"].append(collection_t - data_dict["t"][0])

    # Plot x data
    fig_transx, ax_transx = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
    )

    fig_transx.supxlabel(r"Time $[s]$")

    # add vertical lines for the collection times
    # for collection_t in data_dict["collection_times"]:
    #     plt.axvline(
    #         x=collection_t,
    #         ymin=0,
    #         ymax=1,
    #         color="tab:orange",
    #         linestyle='--',
    #     )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["x"],
        color="r",
        linewidth=3,
        # label="TF Position",
        ax=ax_transx[0],
    )
    # ax2 = ax1.twinx()
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["x"],
        color="g",
        linewidth=3,
        # label="TF Velocity",
        ax=ax_transx[1],
    )
    # ax3 = ax1.twinx()
    # ax3.spines.right.set_position(("axes", 1.1))
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["x"],
        color="b",
        linewidth=3,
        alpha=0.8,
        label="TF Acceleration",
        ax=ax_transx[2],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["lin_accel_imu"]["x"][0::30],
        marker="o",
        color="black",
        s=70,
        label="IMU Acceleration",
        ax=ax_transx[2],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"],
    #     y=data_dict["lin_accel_imu"]["x"],
    #     color="black",
    #     linewidth=2,
    #     alpha=0.4,
    #     label="IMU Acceleration (smoothed)",
    #     ax=ax_transx[2],
    # )

    # Label Handling
    # scatter_1, labels_1 = ax_transx[0].get_legend_handles_labels()
    # scatter_2, labels_2 = ax_transx[1].get_legend_handles_labels()
    # scatter_3, labels_3 = ax_transx[2].get_legend_handles_labels()

    # for i in range(3):
    #     if ax[i].get_legend():
    #         ax[i].get_legend().remove()

    # ax_trans[2].legend(scatter_1 + scatter_2 + scatter_3, labels_1 + labels_2 + labels_3, loc="upper right")

    # Plot y data
    fig_transy, ax_transy = plt.subplots(nrows=3, ncols=1, sharex=True)

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["y"],
        color="r",
        linewidth=3,
        ax=ax_transy[0],
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["y"],
        color="g",
        linewidth=3,
        ax=ax_transy[1],
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["y"],
        color="b",
        linewidth=3,
        alpha=0.8,
        label="TF Acceleration",
        ax=ax_transy[2],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["lin_accel_imu"]["y"][0::30],
        marker="o",
        color="black",
        s=70,
        label="IMU Acceleration",
        ax=ax_transy[2],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"][0::20],
    #     y=data_dict["lin_accel_imu"]["y"][0::20],
    #     color="black",
    #     linewidth=3,
    #     alpha=0.7,
    #     label="IMU Acceleration (smoothed)",
    #     ax=ax_transy[2],
    # )

    # Plot z data
    fig_transz, ax_transz = plt.subplots(
        nrows=3,
        ncols=1,
        sharex=True,
    )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["z"],
        color="r",
        linewidth=3,
        ax=ax_transz[0],
    )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_vel"]["z"],
        color="g",
        linewidth=3,
        ax=ax_transz[1],
    )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["lin_accel"]["z"],
        alpha=0.8,
        color="b",
        linewidth=3,
        label="TF Acceleration",
        ax=ax_transz[2],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["lin_accel_imu"]["z"][0::30],
        marker="o",
        color="black",
        label="IMU Acceleration",
        ax=ax_transz[2],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"][0::20],
    #     y=data_dict["lin_accel_imu"]["z"][0::20],
    #     color="black",
    #     linewidth=3,
    #     alpha=0.7,
    #     label="IMU Acceleration (smoothed)",
    #     ax=ax_transz[2],
    # )

    # Angular Velocity Plots
    fig_rotx, ax_rotx = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["x"],
        color="r",
        linewidth=3,
        ax=ax_rotx[0],
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["x"],
        color="g",
        linewidth=3,
        label="TF Angular Velocity",
        ax=ax_rotx[1],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["ang_vel_imu"]["x"][0::30],
        marker="o",
        color="black",
        label="IMU Angular Velocity",
        ax=ax_rotx[1],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"][0::20],
    #     y=data_dict["ang_vel_imu"]["x"][0::20],
    #     color="black",
    #     linewidth=3,
    #     alpha=0.7,
    #     label="IMU Angular Velocity (smoothed)",
    #     ax=ax_rotx[1],
    # )
    
    # Label Handling
    # scatter_10, labels_10 = ax10.get_legend_handles_labels()
    # scatter_11, labels_11 = ax11.get_legend_handles_labels()

    # for ax in [ax10, ax11]:
    #     if ax.get_legend():
    #         ax.get_legend().remove()

    # ax10.legend(scatter_10 + scatter_11, labels_10 + labels_11, loc="upper right")

    fig_roty, ax_roty = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
    )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["y"],
        color="r",
        linewidth=3,
        ax=ax_roty[0],
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["y"],
        color="g",
        linewidth=3,
        label="TF Angular Velocity",
        ax=ax_roty[1],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["ang_vel_imu"]["y"][0::30],
        marker="o",
        color="black",
        label="IMU Angular Velocity",
        ax=ax_roty[1],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"][0::20],
    #     y=data_dict["ang_vel_imu"]["y"][0::20],
    #     color="black",
    #     linewidth=3,
    #     alpha=0.7,
    #     label="IMU Angular Velocity (smoothed)",
    #     ax=ax_roty[1],
    # )

    # Label Handling
    # scatter_12, labels_12 = ax12.get_legend_handles_labels()
    # scatter_13, labels_13 = ax13.get_legend_handles_labels()

    # for ax in [ax12, ax13]:
    #   if ax.get_legend():
    #         ax.get_legend().remove()

    # ax12.legend(scatter_12 + scatter_13, labels_12 + labels_13, loc="upper right")

    fig_rotz, ax_rotz = plt.subplots(
        nrows=2,
        ncols=1,
        sharex=True,
    )

    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["z"],
        color="r",
        linewidth=3,
        label="TF Angle",
        ax=ax_rotz[0],
    )
    sns.lineplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["z"],
        color="g",
        linewidth=3,
        label="TF Angular Velocity",
        ax=ax_rotz[1],
    )
    sns.scatterplot(
        x=data_dict["t_reparam"][0::30],
        y=data_dict["ang_vel_imu"]["z"][0::30],
        marker="o",
        color="black",
        label="IMU Angular Velocity",
        ax=ax_rotz[1],
    )
    # sns.lineplot(
    #     x=data_dict["t_reparam"][0::20],
    #     y=data_dict["ang_vel_imu"]["z"][0::20],
    #     color="black",
    #     linewidth=3,
    #     alpha=0.7,
    #     label="IMU Angular Velocity (smoothed)",
    #     ax=ax_rotz[1],
    # )

    # Label Handling
    # scatter_14, labels_14 = ax14.get_legend_handles_labels()
    # scatter_15, labels_15 = ax15.get_legend_handles_labels()
    #
    # for ax in [ax14, ax15]:
    #     if ax.get_legend():
    #         ax.get_legend().remove()

    # ax14.legend(scatter_14 + scatter_15, labels_14 + labels_15, loc="upper right")

    fig_angspeed, ax_angspeed = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_speed"],
        marker="o",
        color="r",
        label="Angular speed (derived)",
        ax=ax_angspeed,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_speed_imu"],
        marker="*",
        color="orange",
        label="Angular speed (IMU)",
        ax=ax_angspeed,
    )

    # Add vertical lines showing when the collections were gathered
    # for ax in [ax_transx, ax_transy, ax_transz]:
    #     for collection_t in data_dict["collection_times"]:
    #         ax[2].axvline(
    #             x=collection_t,
    #             ymin=0,
    #             ymax=1,
    #             color="tab:orange",
    #             linestyle="--",
    #         )
    # for ax in [ax_rotx, ax_roty, ax_rotz]:
    #     for collection_t in data_dict["collection_times"]:
    #         ax[1].axvline(
    #             x=collection_t,
    #             ymin=0,
    #             ymax=1,
    #             color="tab:orange",
    #             linestyle="--",
    #         )

    # Some plot formatting
    fig_transx.suptitle(r"Translation ($x$)", fontsize=20)
    fig_transy.suptitle(r"Translation ($y$)", fontsize=20)
    fig_transz.suptitle(r"Translation ($z$)", fontsize=20)
    fig_rotx.suptitle(r"Angular Velocity Data ($\omega_x$)", fontsize=20)
    fig_roty.suptitle(r"Angular Velocity Data ($\omega_y$)", fontsize=20)
    fig_rotz.suptitle(r"Angular Velocity Data ($\omega_z$)", fontsize=20)

    for ax in [ax_transx, ax_transy, ax_transz]:
        ax[0].set_ylabel(ylabel=r"Position $[m]$", color="r", fontsize=16)
        ax[0].set_ylim(-1.5, 1.5)
        ax[1].set_ylabel(ylabel=r"Velocity $[m/s]$", color="g", fontsize=16)
        ax[1].set_ylim(-0.5, 0.5)
        ax[2].set_ylabel(ylabel=r"Acceleration $[m/s^2]$", color="b", fontsize=16)
        ax[2].set_ylim(-0.25, 0.25)

    for ax in [ax_rotx, ax_roty, ax_rotz]:
        ax[0].set_ylabel(ylabel=r"Orientation $[rad]$", color="r", fontsize=16)
        ax[0].set_ylim(-3, 3)
        ax[0].yaxis.label.set_color("r")
        # ax.tick_params(axis="y", colors="r")
        ax[1].set_ylabel(ylabel=r"Angular Velocity $[rad/s]$", color="g", fontsize=16)
        ax[1].set_ylim(-1.5, 1.5)
        ax[1].yaxis.label.set_color("g")
        ax[1].tick_params(axis="y", colors="g")

    ax_angspeed.set(ylabel=r"Angular Speed $[rad/s]$")
    ax_angspeed.set_ylim(-1, 5)

    for fig in [
        fig_transx,
        fig_transy,
        fig_transz,
        fig_rotx,
        fig_roty,
        fig_rotz,
        fig_angspeed,
    ]:
        fig.set_size_inches(18.5, 10.5)
        fig.tight_layout()

    results_folder = os.environ["DERIVATION_RESULTS"]
    output_folder = Path(
        results_folder
        + "/"
        + dataset["_metadata"]["package_name"]
        + "/"
        + dataset_name
        + "/"
        + "noise_"
        + str(noise[0])
        + "_"
        + str(noise[1])
        + "/ns_"
        + str(neighbourhood_size)
        + "_pd_"
        + str(poly_degree)
    )
    output_folder.mkdir(exist_ok=True, parents=True)

    plt.show()

    fig_transx.savefig(str(output_folder) + "/x_trans.png")
    fig_transy.savefig(str(output_folder) + "/y_trans.png")
    fig_transz.savefig(str(output_folder) + "/z_trans.png")
    fig_rotx.savefig(str(output_folder) + "/x_rot.png")
    fig_roty.savefig(str(output_folder) + "/y_rot.png")
    fig_rotz.savefig(str(output_folder) + "/z_rot.png")
    fig_angspeed.savefig(str(output_folder) + "/ang_speed.png")


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
        x=data_dict["t_reparam"],
        y=data_dict["position"]["x"],
        label="x",
        color="r",
        alpha=0.7,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["y"],
        label="y",
        color="g",
        alpha=0.7,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["position"]["z"],
        label="z",
        color="b",
        alpha=0.7,
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
            lin_accel_funcs, lin_vel_funcs, pos_funcs = deriveTranslation(
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
            curve_fitting_colors = ["orange", "green", "cyan"]
            curve_fitting_labels = [
                r"$x$ polynomial curve",
                r"$y$ polynomial curve",
                r"$z$ polynomial curve",
            ]
            for i in range(3):
                pos_func = pos_funcs[i](t_func)
                vel_func = lin_vel_funcs[i](t_func)
                accel_func = lin_accel_funcs[i](t_func)
                p_pos = sns.lineplot(
                    x=t_func_reparam,
                    y=pos_func,
                    ax=ax,
                    linestyle="-",
                    color=curve_fitting_colors[i],
                    label=curve_fitting_labels[i],
                    markersize=50,
                )
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
