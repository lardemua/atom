"""
Utilities for the derivation of TF data
"""

import json
from math import floor
import os
import pathlib
from typing import Any, List, Dict
from matplotlib import pyplot as plt
import seaborn as sns

import numpy as np
from atom_core.atom import getTransform
from atom_core.geometry import matrixToTranslationQuaternion
from scipy.spatial.transform import Rotation


def timeFloatToStamp(t_float: float) -> Dict[str, int]:

    secs = floor(t_float)
    nsecs = round((t_float - secs) * 10**9)

    stamp = {"secs": secs, "nsecs": nsecs}

    return stamp


def timeStampToFloat(stamp: Dict[str, int]) -> float:

    t_float = stamp["secs"] + (10 ** (-9)) * stamp["nsecs"]

    return t_float


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

        # Use dict form for tf

        dict_to_append = {
            "parent": from_frame,
            "child": to_frame,
            "trans": tvec,
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
    """Given a list of transformations, return the functions that describe the angular velocities
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
            [tf["euler"][0] for tf in tf_list],
            [tf["euler"][1] for tf in tf_list],
            [tf["euler"][2] for tf in tf_list],
        ]
    )

    p = [np.polyfit(t_arr, rot_array[i], deg=poly_degree) for i in range(3)]
    p_der = []

    if visualization:
        fig, axes = plt.subplots(2, 3)

    for i in range(3):
        poly_func = np.poly1d(p[i])

        p_der.append(np.polyder(poly_func))
        print(p_der)

        if visualization:
            sns.scatterplot(x=t_arr, y=rot_array[i], ax=axes[0, i])
            x_func = np.linspace(t_arr.min(), t_arr.max(), 1000)
            y_func = poly_func(x_func)
            sns.lineplot(x=x_func, y=y_func, color="red", ax=axes[0, i])

            yder_func = p_der[i](x_func)
            sns.lineplot(x=x_func, y=yder_func, color="green", ax=axes[1, i])

    if visualization:
        plt.show()

    return p_der


def deriveFromTF(
    dataset: dict,
    t: float,
    from_frame: str,
    to_frame: str,
    neighbourhood_size: int,
    poly_degree: int,
    visualization: bool,
) -> List[float]:

    tf_lst = dataset["continuous_data"]["transforms"]

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

    # For each tf, convert quat to axis-angle
    tf_to_derive_lst = convertRotationsInTFToEuler(tf_to_derive_lst)

    print(tf_to_derive_lst)

    ang_vel_funcs = deriveRotation(
        tf_to_derive_lst, poly_degree=poly_degree, visualization=visualization
    )

    print([ang_vel_funcs[i](2661.35) for i in range(3)])

    o = []
    return o


if __name__ == "__main__":

    script_dir = pathlib.Path(__file__).parent

    with open(
        pathlib.Path(os.environ["ATOM_DATASETS"]) / "rihibot/dataset1/dataset.json"
    ) as f:
        input_dataset = json.load(f)

    t = 2661.35
    neighbourhood_size = 5

    first_order_derivatives = deriveFromTF(
        dataset=input_dataset,
        t=t,
        from_frame="world",
        to_frame="imu_link",
        neighbourhood_size=neighbourhood_size,
        poly_degree=51,
        visualization=True,
    )
