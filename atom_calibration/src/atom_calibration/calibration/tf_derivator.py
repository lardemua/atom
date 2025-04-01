"""
Utilities for the derivation of TF data
"""

import json
from math import floor
import pathlib
from typing import Any, List, Dict
from atom_core.atom import getTransform
from atom_core.geometry import matrixToTranslationQuaternion


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


def deriveFromTF(
    dataset: dict, t: float, from_frame: str, to_frame: str, neighbourhood_size: int
) -> List[float]:

    tf_list = dataset["continuous_data"]["transforms"]

    o = []
    return o

if __name__ == '__main__':

    script_dir = pathlib.Path(__file__).parent

    with open(script_dir / "tests/test_data/getTFToDeriveList.json") as f:
        input_dataset = json.load(f)

    t = 2650.636300000
    n = 3

    print(getTFToDeriveList(input_dataset["continuous_data"]["transforms"], "world", "imu_link", t, n))