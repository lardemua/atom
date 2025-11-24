#!/usr/bin/env python3

import argparse
from typing import TextIO, Union
import numpy as np
from numpy._typing import NDArray
from atom_core.geometry import matrixToTranslationRotation
from atom_core.dataset_io import (
    addNoiseToInitialGuess,
    filterCollectionsFromDataset,
    loadResultsJSON,
)
import math
from atom_core.transformations import compareTransforms
from atom_core.atom import getTransform


def getKalibrTF(file_name: str) -> NDArray:

    file: TextIO = open(file_name, "r")

    res_index: Union[None, list[int]] = None
    res_lines: Union[None, list[Union[str, float]]]

    for index, line in enumerate(file):

        if "T_ci" in line:
            res_index = [index + j for j in range(1, 5)]
            res_lines = []

        if res_index and index in res_index:
            res_lines.append(line)

    file.close()

    for idx in range(len(res_lines)):
        res_lines[idx] = res_lines[idx].replace("\n", "")
        res_lines[idx] = res_lines[idx].replace("[", "")
        res_lines[idx] = res_lines[idx].replace("]", "")

        res_lines[idx] = res_lines[idx].split(" ")

        res_lines[idx] = [float(x) for x in res_lines[idx] if x != ""]

    kalibr_cam_imu_tf: NDArray = np.array(res_lines)

    return kalibr_cam_imu_tf


def getAtomTf(
    file_name: str,
    imu_sensor: str,
    camera_sensor: str,
    optical_frame_compensation: bool,
) -> NDArray:

    # Read dataset file
    dataset, _ = loadResultsJSON(file_name, None)  # 2nd arg is csf
    selected_collection_key = list(dataset["collections"].keys())[0]

    tf_pool = dataset["collections"][selected_collection_key]["transforms"]

    # source_frame = dataset["sensors"][camera_sensor]["calibration_child"]
    source_frame = "hand_rgb"
    target_frame = dataset["sensors"][imu_sensor]["calibration_child"]

    res = getTransform(
        from_frame=source_frame,
        to_frame=target_frame,
        transforms=tf_pool,
    )

    if optical_frame_compensation:
        R = res[:3, :3]
        M = np.array(
            [
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
            ]
        )

        R_of = R @ M

        res[:3, :3] = R_of

    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-json",
        "--json_file",
        help="Json file containing ATOM dataset.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-krf",
        "--kalibr_results_file",
        help=".txt file containing kalibr cam-IMU calibration results.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-imu",
        "--imu_sensor",
        help="IMU sensor name.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-c",
        "--camera",
        help="Camera sensor name.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-ofc",
        "--optical_frame_compensation",
        action="store_true",
        default=False,
        help="Rotates camera frame in order to use the typical optical frame representation (Z+ aligned with optical axis). Use if your system does not have the optical frame defined. This might be needed if, for example, you use Webots for simulation.",
    )

    # - Save args
    args = vars(ap.parse_known_args()[0])

    kalibr_res = getKalibrTF(args["kalibr_results_file"])

    gt = getAtomTf(
        file_name=args["json_file"],
        imu_sensor=args["imu_sensor"],
        camera_sensor=args["camera"],
        optical_frame_compensation=args["optical_frame_compensation"],
    )
    translation_error, rotation_error, _, _, _, _, _, _ = compareTransforms(
        kalibr_res, gt
    )

    print("Etrans = " + str(round(translation_error * 1000, 3)) + " (mm)")
    print("Erot = " + str(round(rotation_error * 180 / math.pi, 3)) + " (deg)")


if __name__ == "__main__":
    main()
