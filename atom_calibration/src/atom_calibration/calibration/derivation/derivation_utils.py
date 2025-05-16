#!/usr/bin/env python3

from copy import deepcopy
from math import floor
from typing import Dict, List

import numpy as np
import seaborn as sns
from atom_core.atom import getTransform
from atom_core.geometry import matrixToTranslationQuaternion
from atom_core.utilities import atomError
from matplotlib import pyplot as plt
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
        for tf in dataset["continuous_data"]["/tf_static"][-1]["transforms"]:
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


def centralNumericalFirstDerivative(data: Dict[str, float]) -> List[float]:
    """
    Numerically calculates the first derivative at instant t_i using central derivation.

    Input:
        data: a dictionary of the relevant data for derivation. The dictionary must have the following structure:

        data = {
            "t": [
                t_{i-1},
                t_i,
                t_{i+1},
                ],
            "var1": [
                var1(t_{i-1}),
                var(t_i),
                var(t_{i+1}),
                ],
            ....
            "varN": [
                varN(t_{i-1}),
                varN(t_i),
                varN(t_{i+1}),
                ]
            }

    Output:
        ddata_dt: a list with the derivatives at instant t, in order of the variables of the input dictionary

        ddata_dt = [
            dvar1_dt(t_i),
            ...,
            dvarN_dt(t_i)
            ]
    """
    ddata_dt = []

    for var in data.keys():
        if var == "t":
            continue

        dvar_dt = (data[var][2] - data[var][0]) / (data["t"][2] - data["t"][0])
        ddata_dt.append(dvar_dt)

    return ddata_dt


def centralNumericalSecondDerivative(data: Dict[str, float]) -> List[float]:
    """
    Numerically calculates the second derivative at instant t_i using central derivation.

    Input:
        data: a dictionary of the relevant data for derivation. Uses 3 datapoints, where t_i is the central instant. The dictionary must have the following structure:

        data = {
            "t": [
                t_{i-1},
                t_i,
                t_{i+1},
                ],
            "var1": [
                var1(t_{i-1}),
                var(t_i),
                var(t_{i+1}),
                ],
            ....
            "varN": [
                varN(t_{i-1}),
                varN(t_i),
                varN(t_{i+1}),
                ]
            }

    Output:
        dddata_dtt: a list with the derivatives at instant t, in order of the variables of the input dictionary

        dddata_dtt = [
            ddvar1_dtt(t_i),
            ...,
            ddvarN_dtt(t_i)
            ]
    """
    dddata_dtt = []

    for var in data.keys():
        if var == "t":
            continue

        ddvar_dtt = (data[var][2] - 2 * data[var][1] + data[var][0]) / (
            (data["t"][2] - data["t"][1]) ** 2
        )
        dddata_dtt.append(ddvar_dtt)

    return dddata_dtt


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
    transition_point_list: List,
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

    # Remove from the list tfs with the stamp before/after transition points based on where it is in relation to t
    for tf in source_to_target_tf_lst:
        tf_t = timeStampToFloat(tf["stamp"])
        if tf_t in transition_point_list:
            if tf_t <= t:
                source_to_target_tf_lst = [
                    x
                    for x in source_to_target_tf_lst
                    if not timeStampToFloat(x["stamp"]) <= tf_t
                ]
            elif tf_t >= t:
                source_to_target_tf_lst = [
                    x
                    for x in source_to_target_tf_lst
                    if not timeStampToFloat(x["stamp"]) >= tf_t
                ]

    return source_to_target_tf_lst


def convertRotationsInTFToEuler(tf_list: List[Dict]) -> List[Dict]:
    """Convert the rotations in a list of TFs to euler angles (XYZ convention)"""

    for tf in tf_list:
        r = Rotation.from_quat(tf["quat"])
        r_euler = r.as_euler(seq="XYZ")

        tf["euler"] = r_euler.tolist()

    return tf_list
