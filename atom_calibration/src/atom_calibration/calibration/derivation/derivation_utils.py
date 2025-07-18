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
    for tf_msg in dataset["continuous_data"]["/tf"]:

        tf_dict_to_append = {}

        # Get stamp from one of the transforms in the tf_msg
        # NOTE: Since all of the TFs in the same "transforms" field have the same stamp, I can just access the first one
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
        for tf in dataset["continuous_data"]["/tf_static"][0]["transforms"]:
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


def plotDerivationResults(
    dataset: Dict,
    tf_list: List,
    derivation_results: Dict,
    from_frame: str,
    to_frame: str,
    noise: tuple,
    dataset_name: str,
    ignore_gravity: bool,
) -> None:

    # NOTE: It doesn't make sense to plot out orientation since it's expressed in quaternions
    data_dict = {
        "t": [],
        "t_reparam": [],
        "position": {"x": [], "y": [], "z": []},
        "lin_vel": {"x": [], "y": [], "z": []},
        "lin_accel": {"x": [], "y": [], "z": []},
        "lin_accel_imu": {"x": [], "y": [], "z": []},
        "angs": {"x": [], "y": [], "z": []},
        "ang_vel": {"x": [], "y": [], "z": []},
        "ang_vel_imu": {"x": [], "y": [], "z": []},
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

        # Get closest acceleration data
        closest_imu_datapoint = min(
            dataset["continuous_data"]["/imu"],
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

        # Norm of velocity vectors
        imu_ang_speed = np.linalg.norm(imu_ang_vel)

        # Convert to Rotation instance and re-convert to match to derivative
        imu_ang_vel_r = Rotation.from_rotvec(imu_ang_vel)
        imu_ang_vel = imu_ang_vel_r.as_matrix()
        imu_ang_vel_r = Rotation.from_matrix(imu_ang_vel)
        imu_ang_vel = imu_ang_vel_r.as_rotvec()

        R = world_imu_tf[:3, :3]
        imu_accel = R @ imu_accel
        if not ignore_gravity:
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

        data_dict["lin_accel_imu"]["x"].append(imu_accel[0])
        data_dict["lin_accel_imu"]["y"].append(imu_accel[1])
        data_dict["lin_accel_imu"]["z"].append(imu_accel[2])

        data_dict["ang_vel_imu"]["x"].append(imu_ang_vel[0])
        data_dict["ang_vel_imu"]["y"].append(imu_ang_vel[1])
        data_dict["ang_vel_imu"]["z"].append(imu_ang_vel[2])

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
        label="IMU Acceleration data",
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
        label="IMU Acceleration data",
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
        label="IMU Acceleration data",
        ax=ax9,
    )

    # Angular Velocity Plots
    fig4, ax10 = plt.subplots()
    ax11 = ax10.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["x"],
        marker="o",
        color="r",
        ax=ax10,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["x"],
        marker="s",
        color="g",
        label="Derivation Results",
        ax=ax11,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel_imu"]["x"],
        marker="*",
        color="orange",
        label="IMU Angular Velocity Data",
        ax=ax11,
    )

    fig5, ax12 = plt.subplots()
    ax13 = ax12.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["y"],
        marker="o",
        color="r",
        ax=ax12,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["y"],
        marker="s",
        color="g",
        label="Derivation Results",
        ax=ax13,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel_imu"]["y"],
        marker="*",
        color="orange",
        label="IMU Angular Velocity Data",
        ax=ax13,
    )
    fig6, ax14 = plt.subplots()
    ax15 = ax14.twinx()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["angs"]["z"],
        marker="o",
        color="r",
        ax=ax14,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel"]["z"],
        marker="s",
        color="g",
        label="Derivation Results",
        ax=ax15,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_vel_imu"]["z"],
        marker="*",
        color="orange",
        label="IMU Angular Velocity Data",
        ax=ax15,
    )
    fig7, ax16 = plt.subplots()
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_speed"],
        marker="o",
        color="r",
        label="Angular speed (derived)",
        ax=ax16,
    )
    sns.scatterplot(
        x=data_dict["t_reparam"],
        y=data_dict["ang_speed_imu"],
        marker="*",
        color="orange",
        label="Angular speed (IMU)",
        ax=ax16,
    )
    
    # Some plot formatting
    ax1.set_title(r"Translation Data ($x$)")
    ax4.set_title(r"Translation Data ($y$)")
    ax7.set_title(r"Translation Data ($z$)")
    ax10.set_title(r"Angular Velocity Data ($\omega_x$)")
    ax12.set_title(r"Angular Velocity Data ($\omega_y$)")
    ax14.set_title(r"Angular Velocity Data ($\omega_z$)")

    for ax in [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10]:
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
    for ax in [ax10, ax12, ax14]:
        ax.set(ylabel=r"Orientation $[rad]$")
        ax.set_ylim(-3, 3)
        ax.yaxis.label.set_color("r")
    for ax in [ax11, ax13, ax15]:
        ax.set(ylabel=r"Angular Velocity $[rad/s]$")
        ax.set_ylim(-1.5, 1.5)
        ax.yaxis.label.set_color("g")

    ax16.set(ylabel=r"Angular Speed $[rad/s]$")
    ax16.set_ylim(-1,5)

    for fig in [fig1, fig2, fig3, fig4, fig5, fig6, fig7]:
        fig.set_size_inches(18.5, 10.5)
        fig.tight_layout()

    results_folder = os.environ["DERIVATION_RESULTS"]
    output_folder = Path(
        results_folder
        + "/"
        + dataset_name
        + "/"
        + "noise_"
        + str(noise[0])
        + "_"
        + str(noise[1])
    )
    output_folder.mkdir(exist_ok=True, parents=True)

    fig1.savefig(str(output_folder) + "/x_trans.png")
    fig2.savefig(str(output_folder) + "/y_trans.png")
    fig3.savefig(str(output_folder) + "/z_trans.png")
    fig4.savefig(str(output_folder) + "/x_rot.png")
    fig5.savefig(str(output_folder) + "/y_rot.png")
    fig6.savefig(str(output_folder) + "/z_rot.png")
    fig7.savefig(str(output_folder) + "/ang_speed.png")


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
