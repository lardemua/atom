"""
Utilities for the derivation of TF data
"""

import json
import math
import os
import pathlib
from math import floor
from pprint import pprint
from typing import Any, Dict, List

import numpy as np
import seaborn as sns
from atom_core.atom import getTransform
from atom_core.geometry import matrixToTranslationQuaternion
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
        tf_dict_to_append["stamp"] = tf_msg["transforms"][0]["header"]["stamp"]

        for tf in tf_msg["transforms"]:
            child_frame = tf["child_frame_id"]
            parent_frame = tf["header"]["frame_id"]
            key = f"{parent_frame}-{child_frame}"

            tf_dict_to_append[key] = {
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

            tf_dict_to_append[key] = {
                "child": child_frame,
                "parent": parent_frame,
                "quat": [*tf["transform"]["rotation"].values()],
                "trans": [*tf["transform"]["translation"].values()],
            }

        tf_list.append(tf_dict_to_append)

    return tf_list


def quatMult(q: List, p: List) -> List:

    res = [
        p[0] * q[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3],
        q[1] * p[0] + q[0] * p[1] + q[2] * p[3] - q[3] * p[2],
        q[2] * p[0] + q[0] * p[2] + q[3] * p[1] - q[1] * p[3],
        q[3] * p[0] + q[0] * p[3] + q[1] * p[2] - q[2] * p[1],
    ]

    return res


def getTFToDeriveList(
    tf_list: List[Any], from_frame: str, to_frame: str, t: float, n: int
) -> List[Dict]:
    """Get a list of n source-target TFs in the temporal neighbourhood of timestamp."""

    # Find the n tf dicts (full topological tree) closest to the timestamp
    closest_tf_dicts = []

    for i in range(n):
        min_t_dist = None
        min_element = None
        for element in tf_list:
            element_t = timeStampToFloat(element["stamp"])
            t_dist = abs(t - element_t)

            # if element_t <= t and (min_element is None or t_dist < min_t_dist):
            if min_element is None or t_dist < min_t_dist:
                min_t_dist = t_dist
                min_element = element

        closest_tf_dicts.append(min_element)
        tf_list.remove(min_element)

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

    q = [np.polyfit(t_arr, trans_array[i], deg=poly_degree) for i in range(3)]
    dq = []
    ddq = []

    if visualization:
        fig, axes = plt.subplots(3, 3)

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
        plt.show()

    lin_accel = [ddq[0], ddq[1], ddq[2]]

    return lin_accel


def deriveFromTF(
    dataset: dict,
    t: float,
    from_frame: str,
    to_frame: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
) -> List[float]:
    """Given a dataset and a timestamp t, return the results of derivation for that instant of time."""

    tf_lst = getTFList(dataset)

    # DEBUG
    with open(
        "/home/diogo/catkin_ws/src/atom/atom_calibration/src/atom_calibration/test.json",
        "w",
    ) as f:
        json.dump(tf_lst, f)

    # Get a list of the n temporally closest (wrt t) tfs to use for derivation
    tf_to_derive_lst = getTFToDeriveList(
        tf_list=tf_lst,
        from_frame=from_frame,
        to_frame=to_frame,
        t=t,
        n=neighbourhood_size,
    )

    # Sort tf_to_derive_list according to timestamp
    tf_to_derive_lst = sorted(
        tf_to_derive_lst, key=lambda x: timeStampToFloat(x["stamp"])
    )

    lin_accel_funcs = deriveTranslation(
        tf_list=tf_to_derive_lst, poly_degree=poly_degree, visualization=visualization
    )

    lin_accel = []
    for i in range(3):
        tmp_f = lin_accel_funcs[i]
        lin_accel.append(tmp_f(t))

    # DISABLING TEMPORARILY
    # ang_vel_funcs = deriveRotation(
    #     tf_to_derive_lst, poly_degree=poly_degree, visualization=visualization
    # )

    # print([ang_vel_funcs[i](2660.657) for i in range(3)])
    ang_vel = [0, 0, 0]

    return lin_accel, ang_vel


def calculateErrors(
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
            "e_lin_accel": np.linalg.norm(imu_accel - results["lin_accel"]),
            "e_ang_vel": "NaN",
        }

    return e


def deriveDataset(
    dataset: dict,
    from_frame: str,
    to_frame: str,
    sensor_name: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
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
        )

        derivation_results[collection_key] = {
            "lin_accel": lin_accel,
            "ang_vel": ang_vel,
        }

    pprint(derivation_results)

    # Calculate errors
    e = calculateErrors(
        dataset=dataset,
        results=derivation_results,
        sensor_name=sensor_name,
        from_frame=from_frame,
        to_frame=to_frame,
    )

    # Print error table
    e_table = PrettyTable()
    e_table.field_names = ["Collection", "E_lin_accel (m/s^2)", "E_ang_vel (rad/s)"]

    e_table.add_rows(
        [
            [
                collection_key,
                round(float(e[collection_key]["e_lin_accel"]), 4),
                round(float(e[collection_key]["e_ang_vel"]), 4),
            ]
            for collection_key in e.keys()
        ]
    )

    print(e_table)

    return 0


if __name__ == "__main__":

    script_dir = pathlib.Path(__file__).parent

    with open(
        pathlib.Path(os.environ["ATOM_DATASETS"]) / "rihibot/dataset1/dataset.json"
    ) as f:
        input_dataset = json.load(f)

    neighbourhood_size = 75

    derivation_results = deriveDataset(
        dataset=input_dataset,
        from_frame="world",
        to_frame="imu_link",
        sensor_name="imu_hand",
        neighbourhood_size=neighbourhood_size,
        poly_degree=2,
        visualization=True,
    )

    # first_order_derivatives = deriveFromTF(
    #     dataset=input_dataset,
    #     t=t,
    #     from_frame="world",
    #     to_frame="imu_link",
    #     neighbourhood_size=neighbourhood_size,
    #     poly_degree=2,
    #     visualization=True,
    # )
